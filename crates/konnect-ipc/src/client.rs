//! KiCAD 10 IPC API client using NNG + Protocol Buffers.
//!
//! KiCAD 10 exposes an IPC API over NNG (nanomsg-next-gen) using protobuf messages.
//! The transport is NNG req/rep over IPC (Unix sockets / Windows named pipes).
//!
//! Socket path: set by KICAD_API_SOCKET env var when KiCAD launches a plugin,
//! or can be manually specified.
//!
//! Protocol: ApiRequest envelope containing a google.protobuf.Any body → ApiResponse.

use crate::gen::kiapi;
use crate::types::*;
use anyhow::{Context, Result};
// NNG SetOpt trait is brought in scope automatically by the nng crate's prelude
use prost::Message;
use tracing::{debug, warn};

/// Converts KiCAD nanometers to millimeters.
fn nm_to_mm(nm: i64) -> f64 {
    nm as f64 / 1_000_000.0
}

/// Map a BoardLayer enum integer back to a KiCAD layer name string.
fn layer_enum_to_name(layer: i32) -> &'static str {
    match kiapi::board::types::BoardLayer::try_from(layer) {
        Ok(l) => match l {
            kiapi::board::types::BoardLayer::BlFCu => "F.Cu",
            kiapi::board::types::BoardLayer::BlBCu => "B.Cu",
            kiapi::board::types::BoardLayer::BlIn1Cu => "In1.Cu",
            kiapi::board::types::BoardLayer::BlIn2Cu => "In2.Cu",
            kiapi::board::types::BoardLayer::BlFSilkS => "F.SilkS",
            kiapi::board::types::BoardLayer::BlBSilkS => "B.SilkS",
            kiapi::board::types::BoardLayer::BlFMask => "F.Mask",
            kiapi::board::types::BoardLayer::BlBMask => "B.Mask",
            kiapi::board::types::BoardLayer::BlFPaste => "F.Paste",
            kiapi::board::types::BoardLayer::BlBPaste => "B.Paste",
            kiapi::board::types::BoardLayer::BlFCrtYd => "F.CrtYd",
            kiapi::board::types::BoardLayer::BlBCrtYd => "B.CrtYd",
            kiapi::board::types::BoardLayer::BlFFab => "F.Fab",
            kiapi::board::types::BoardLayer::BlBFab => "B.Fab",
            kiapi::board::types::BoardLayer::BlEdgeCuts => "Edge.Cuts",
            _ => "Unknown",
        },
        Err(_) => "Unknown",
    }
}

/// Wrap a protobuf message into a prost_types::Any with the correct type_url.
fn pack_any<M: Message>(msg: &M, type_name: &str) -> prost_types::Any {
    let mut buf = Vec::new();
    msg.encode(&mut buf).expect("protobuf encode failed");
    prost_types::Any {
        type_url: format!("type.googleapis.com/{}", type_name),
        value: buf,
    }
}

/// Decode a prost_types::Any into a specific protobuf message type.
fn unpack_any<M: Message + Default>(any: &prost_types::Any) -> Result<M> {
    M::decode(any.value.as_slice()).context("Failed to decode protobuf Any body")
}

/// Check the request-level ItemRequestStatus that CreateItems/UpdateItems/
/// DeleteItems responses carry. KiCAD's own proto comments warn this can be
/// IRS_OK even when zero items were actually touched — real success also
/// requires checking the per-item results (see `check_item_status` /
/// `check_deletion_status`), which is why callers check both.
fn check_item_request_status(status: i32, op: &str) -> Result<()> {
    let code = kiapi::common::types::ItemRequestStatus::try_from(status)
        .unwrap_or(kiapi::common::types::ItemRequestStatus::IrsUnknown);
    if code != kiapi::common::types::ItemRequestStatus::IrsOk {
        anyhow::bail!("{} request failed: {}", op, code.as_str_name());
    }
    Ok(())
}

/// Check one item's ItemStatus, as returned per-item by CreateItems/UpdateItems.
fn check_item_status(status: Option<&kiapi::common::commands::ItemStatus>, op: &str) -> Result<()> {
    let status = match status {
        Some(s) => s,
        None => return Ok(()),
    };
    let code = kiapi::common::commands::ItemStatusCode::try_from(status.code)
        .unwrap_or(kiapi::common::commands::ItemStatusCode::IscUnknown);
    if code != kiapi::common::commands::ItemStatusCode::IscOk {
        let detail = if status.error_message.is_empty() {
            code.as_str_name().to_string()
        } else {
            status.error_message.clone()
        };
        anyhow::bail!("{} failed for an item: {}", op, detail);
    }
    Ok(())
}

/// Check one item's ItemDeletionStatus, as returned per-item by DeleteItems.
fn check_deletion_status(status: i32, op: &str) -> Result<()> {
    let code = kiapi::common::commands::ItemDeletionStatus::try_from(status)
        .unwrap_or(kiapi::common::commands::ItemDeletionStatus::IdsUnknown);
    if code != kiapi::common::commands::ItemDeletionStatus::IdsOk {
        anyhow::bail!("{} failed for an item: {}", op, code.as_str_name());
    }
    Ok(())
}

/// Confirm KiCAD reported a result for every item requested. Confirmed live
/// (2026-08-04): for a nonexistent/unresolvable item KiCAD does NOT include
/// an error-status entry for it — it just omits it from the results array
/// entirely, with the request-level status still IRS_OK. Per-item status
/// checks alone can't catch that, since there's no per-item entry to check;
/// a result-count mismatch is the only signal.
fn check_result_count(requested: usize, returned: usize, op: &str) -> Result<()> {
    if returned < requested {
        anyhow::bail!(
            "{} requested {} item(s) but KiCAD only returned a result for {} — \
             the rest don't exist (or are otherwise unresolvable) on the board",
            op,
            requested,
            returned
        );
    }
    Ok(())
}

/// Process-wide lock serializing every save→splice→revert sequence
/// (`add_via`, `place_footprint`). Two overlapping sequences both splice
/// into the same board file via a non-atomic read-modify-write
/// (`splice_sexp_into_string` + `fs::write`) — without this, one can tear
/// the other's write mid-flight. Held for the whole sequence, not just the
/// write, since the read (save) and the write both need to be atomic with
/// respect to each other across calls.
static SAVE_SPLICE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Insert an S-expression block as a top-level sibling of an in-memory
/// `.kicad_pcb` document's contents, just before the closing paren, and
/// return the spliced result. In-memory sibling of the old
/// `splice_sexp_into_board` (file-path version, no longer used — see
/// `save_splice_revert`): same trim/rfind(')')/splice logic, but operating
/// on an owned `String` instead of reading/writing a file directly, so it
/// works whether the content came from disk (the save_board() fallback
/// path) or straight over IPC (save_board_to_string()).
///
/// KiCAD's own `.kicad_pcb` format is a flat list of top-level clauses
/// (`(footprint ...)`, `(via ...)`, `(gr_...)`, ...) inside one outer
/// `(kicad_pcb ...)` s-expression, and KiCAD's own generator always emits
/// the final `)` as the last non-whitespace character, so targeted string
/// splicing (rather than a full rewrite via a S-expression parser) is safe
/// here — but only once the input is confirmed to actually be a
/// well-formed `.kicad_pcb` document. Both the input and the spliced
/// output are parsed and checked for a `kicad_pcb` head before/after
/// splicing; a truncated/torn read (or a splice that broke the file
/// structure) must never proceed to being written back to disk.
fn splice_sexp_into_string(contents: &str, sexp_block: &str) -> Result<String> {
    let head = |s: &str| {
        konnect_sexp::parse_sexp(s)
            .ok()
            .and_then(|n| n.head().map(str::to_string))
    };

    if head(contents).as_deref() != Some("kicad_pcb") {
        anyhow::bail!(
            "refusing to splice: board contents do not parse as a valid .kicad_pcb document \
             (got head {:?}) — likely a truncated or torn read",
            head(contents)
        );
    }

    let trimmed = contents.trim_end();
    let last_paren = trimmed
        .rfind(')')
        .ok_or_else(|| anyhow::anyhow!("board contents have no closing paren"))?;
    let mut out = String::with_capacity(contents.len() + sexp_block.len() + 8);
    out.push_str(&trimmed[..last_paren]);
    out.push('\n');
    out.push_str(sexp_block);
    out.push('\n');
    out.push_str(&trimmed[last_paren..]);
    out.push('\n');

    if head(&out).as_deref() != Some("kicad_pcb") {
        anyhow::bail!(
            "refusing to write spliced board: result no longer parses as a valid .kicad_pcb \
             document after splicing (got head {:?})",
            head(&out)
        );
    }

    Ok(out)
}

pub struct KiCadIpcClient {
    socket_path: String,
    client_name: String,
}

impl KiCadIpcClient {
    /// Create a client connecting to the given IPC socket path.
    /// If empty, tries the KICAD_API_SOCKET environment variable, then (on
    /// Unix) KiCAD's default socket path if it currently exists.
    pub fn new(socket_path: impl Into<String>) -> Self {
        let path = socket_path.into();
        let effective_path = if path.is_empty() {
            std::env::var("KICAD_API_SOCKET").unwrap_or_else(|_| Self::default_unix_socket())
        } else {
            path
        };
        KiCadIpcClient {
            socket_path: effective_path,
            client_name: format!("konnect-{}", std::process::id()),
        }
    }

    /// KiCAD's default API socket on macOS/Linux, existence-gated so the
    /// "IPC not configured" guidance still fires when KiCAD isn't running
    /// or its API server is disabled.
    fn default_unix_socket() -> String {
        #[cfg(unix)]
        {
            let default = "/tmp/kicad/api.sock";
            if std::path::Path::new(default).exists() {
                return format!("ipc://{default}");
            }
        }
        String::new()
    }

    /// Send a protobuf command and return the response Any.
    fn send_command(
        &self,
        command: &impl Message,
        type_name: &str,
    ) -> Result<Option<prost_types::Any>> {
        if self.socket_path.is_empty() {
            anyhow::bail!(
                "KiCAD IPC socket path not configured. To fix: \
                 (1) in KiCAD, enable Edit > Preferences > Plugins > 'Enable KiCad API' \
                 and copy the listed ipc:// address; \
                 (2) paste it into the 'IPC Socket' field of the Konnect settings dialog \
                 (Tools > External Plugins > Konnect) and save; \
                 (3) restart the AI client so the server rereads settings. \
                 Alternatively set ipc_socket_path in konnect-settings.json or launch \
                 via KiCAD (which sets KICAD_API_SOCKET). \
                 Full guide: https://github.com/mixelpixx/Konnect/blob/main/docs/TROUBLESHOOTING.md"
            );
        }

        let request = kiapi::common::ApiRequest {
            header: Some(kiapi::common::ApiRequestHeader {
                kicad_token: String::new(), // Empty = accept any instance
                client_name: self.client_name.clone(),
            }),
            message: Some(pack_any(command, type_name)),
        };

        let request_bytes = request.encode_to_vec();
        debug!(
            "[BETA] IPC → {} ({} bytes) to {}",
            type_name,
            request_bytes.len(),
            self.socket_path
        );

        // Connect via NNG req0 socket
        let socket =
            nng::Socket::new(nng::Protocol::Req0).context("Failed to create NNG socket")?;

        // Bound every step: a busy or wedged KiCAD must produce an error the
        // tools can surface, never an indefinite hang (the predecessor
        // project's sync/autoroute hangs blocked for >600 s on exactly this).
        // 30 s receive allows slow board operations like zone refills.
        use nng::options::Options;
        socket
            .set_opt::<nng::options::SendTimeout>(Some(std::time::Duration::from_secs(5)))
            .context("Failed to set NNG send timeout")?;
        socket
            .set_opt::<nng::options::RecvTimeout>(Some(std::time::Duration::from_secs(30)))
            .context("Failed to set NNG receive timeout")?;

        // Build the dial URL
        let dial_url =
            if self.socket_path.starts_with("ipc://") || self.socket_path.starts_with("tcp://") {
                self.socket_path.clone()
            } else {
                format!("ipc://{}", self.socket_path)
            };

        // Retry the dial briefly: each call opens a brand-new socket rather than
        // reusing a persistent connection, and a momentarily-busy listener can
        // refuse a connection attempt that would succeed a beat later. Chains of
        // several calls in a row (as the round-trip re-queries added throughout
        // this file now do) make that transient window more likely to get hit at
        // least once, not less reliable overall — worth absorbing with a couple
        // of short retries rather than failing the whole operation on it.
        let mut dial_result = socket.dial(&dial_url);
        for attempt in 0..2 {
            if dial_result.is_ok() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50 * (attempt + 1)));
            dial_result = socket.dial(&dial_url);
        }
        dial_result.with_context(|| format!("Cannot connect to KiCAD IPC at {}", dial_url))?;

        // Send request
        let msg = nng::Message::from(request_bytes.as_slice());
        socket
            .send(msg)
            .map_err(|(_, e)| anyhow::anyhow!("NNG send failed: {}", e))?;

        // Receive response
        let reply = socket
            .recv()
            .map_err(|e| anyhow::anyhow!("NNG recv failed: {}", e))?;

        let response = kiapi::common::ApiResponse::decode(reply.as_slice())
            .context("Failed to decode ApiResponse")?;

        // Check status
        if let Some(ref status) = response.status {
            let code = status.status();
            if code != kiapi::common::ApiStatusCode::AsOk {
                let msg = if status.error_message.is_empty() {
                    format!("{:?}", code)
                } else {
                    status.error_message.clone()
                };
                debug!("[BETA] IPC ← error: {} ({})", msg, code.as_str_name());
                anyhow::bail!("KiCAD IPC error: {} ({})", msg, code.as_str_name());
            }
        }

        debug!("[BETA] IPC ← OK");
        Ok(response.message)
    }

    /// Send a mutating command wrapped in a KiCAD IPC commit transaction.
    ///
    /// KiCAD's IPC API requires every board edit to be bracketed with
    /// BeginCommit/EndCommit(commit) — without it, KiCAD accepts the command
    /// (the call returns Ok) but never actually applies/persists it, so the
    /// edit silently vanishes. Every mutating call (CreateItems, UpdateItems,
    /// DeleteItems, RefillZones, ParseAndCreateItemsFromString) must go
    /// through this instead of calling `send_command` directly.
    fn send_mutating_command(
        &self,
        command: &impl Message,
        type_name: &str,
        description: &str,
    ) -> Result<Option<prost_types::Any>> {
        let commit_id = self.begin_commit()?;
        match self.send_command(command, type_name) {
            Ok(resp) => {
                if let Err(err) = self.push_commit(&commit_id, description) {
                    // A failed push leaves the commit open server-side
                    // (commits are keyed by client name), which permanently
                    // fails every subsequent mutating call from this client
                    // until pcbnew restarts. Drop it so the client recovers.
                    let _ = self.drop_commit(&commit_id);
                    return Err(err);
                }
                Ok(resp)
            }
            Err(err) => {
                let _ = self.drop_commit(&commit_id);
                Err(err)
            }
        }
    }

    // ─── Public API (same interface as before, tools don't change) ───────

    /// Check if KiCAD is reachable.
    pub fn ping(&self) -> Result<bool> {
        let ping = kiapi::common::commands::Ping {};
        match self.send_command(&ping, "kiapi.common.commands.Ping") {
            Ok(_) => Ok(true),
            Err(e) => {
                warn!("[BETA] Ping failed: {}", e);
                Ok(false)
            }
        }
    }

    /// Get the list of open documents (boards).
    pub fn get_open_documents(&self) -> Result<Vec<kiapi::common::types::DocumentSpecifier>> {
        let cmd = kiapi::common::commands::GetOpenDocuments {
            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
        };
        let response_any = self.send_command(&cmd, "kiapi.common.commands.GetOpenDocuments")?;
        if let Some(any) = response_any {
            let resp: kiapi::common::commands::GetOpenDocumentsResponse = unpack_any(&any)?;
            Ok(resp.documents)
        } else {
            Ok(vec![])
        }
    }

    /// Get the first open PCB's DocumentSpecifier (needed for most commands).
    fn get_board_document(&self) -> Result<kiapi::common::types::DocumentSpecifier> {
        let docs = self.get_open_documents()?;
        docs.into_iter().next().ok_or_else(|| {
            anyhow::anyhow!("No PCB document is open in KiCAD. Open a board file first.")
        })
    }

    fn make_header(&self) -> Result<kiapi::common::types::ItemHeader> {
        Ok(kiapi::common::types::ItemHeader {
            document: Some(self.get_board_document()?),
            container: None,
            field_mask: None,
        })
    }

    /// Get all nets on the board.
    pub fn get_nets(&self) -> Result<Vec<IpcNet>> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::GetNets {
            board: Some(doc),
            netclass_filter: vec![],
        };
        let response_any = self.send_command(&cmd, "kiapi.board.commands.GetNets")?;
        if let Some(any) = response_any {
            let resp: kiapi::board::commands::NetsResponse = unpack_any(&any)?;
            Ok(resp
                .nets
                .iter()
                .map(|n| IpcNet {
                    name: n.name.clone(),
                    netcode: n.code.as_ref().map(|c| c.value).unwrap_or(0),
                })
                .collect())
        } else {
            Ok(vec![])
        }
    }

    /// Get board items by type.
    pub fn get_items(
        &self,
        item_type: kiapi::common::types::KiCadObjectType,
    ) -> Result<Vec<prost_types::Any>> {
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::GetItems {
            header: Some(header),
            types: vec![item_type as i32],
        };
        let response_any = self.send_command(&cmd, "kiapi.common.commands.GetItems")?;
        if let Some(any) = response_any {
            let resp: kiapi::common::commands::GetItemsResponse = unpack_any(&any)?;
            Ok(resp.items)
        } else {
            Ok(vec![])
        }
    }

    /// List all footprints on the board.
    pub fn list_footprints(&self) -> Result<Vec<IpcFootprint>> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        let mut footprints = Vec::new();
        for item in &items {
            if let Ok(fp) = kiapi::board::types::FootprintInstance::decode(item.value.as_slice()) {
                let pos = fp.position.as_ref();
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.clone())
                    .unwrap_or_default();
                let val_text = fp
                    .value_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.clone())
                    .unwrap_or_default();
                let lib_id = fp
                    .definition
                    .as_ref()
                    .and_then(|d| d.id.as_ref())
                    .map(|id| format!("{}:{}", id.library_nickname, id.entry_name))
                    .unwrap_or_default();
                footprints.push(IpcFootprint {
                    reference: ref_text,
                    value: val_text,
                    footprint: lib_id,
                    position: IpcVector2 {
                        x: pos.map(|p| nm_to_mm(p.x_nm)).unwrap_or(0.0),
                        y: pos.map(|p| nm_to_mm(p.y_nm)).unwrap_or(0.0),
                    },
                    rotation: fp
                        .orientation
                        .as_ref()
                        .map(|a| a.value_degrees)
                        .unwrap_or(0.0),
                    layer: layer_enum_to_name(fp.layer).to_string(),
                    kiid: fp.id.as_ref().map(|k| k.value.clone()).unwrap_or_default(),
                });
            }
        }
        Ok(footprints)
    }

    /// Create items on the board. Checks the request-level status, that
    /// KiCAD actually reported a result for every item requested (KiCAD
    /// silently omits items it couldn't create from the results array
    /// rather than reporting a per-item error for them — confirmed live:
    /// an unresolvable item comes back as an empty results array with an
    /// otherwise-OK request status), each item's own creation result, and —
    /// on top of trusting CreateItems' own response — independently
    /// re-resolves each created item by KIID via GetItemsById afterward
    /// (see `confirm_items_exist`), the same round-trip principle already
    /// applied to delete_items/place_footprint/add_via.
    pub fn create_items(&self, items: Vec<prost_types::Any>) -> Result<()> {
        let requested = items.len();
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::CreateItems {
            header: Some(header),
            items,
            container: None,
        };
        let resp_any =
            self.send_mutating_command(&cmd, "kiapi.common.commands.CreateItems", "Create items")?
                .ok_or_else(|| anyhow::anyhow!("CreateItems returned no response body"))?;
        let resp: kiapi::common::commands::CreateItemsResponse = unpack_any(&resp_any)?;
        check_item_request_status(resp.status, "CreateItems")?;
        check_result_count(requested, resp.created_items.len(), "CreateItems")?;
        let mut kiids = Vec::with_capacity(resp.created_items.len());
        for result in &resp.created_items {
            check_item_status(result.status.as_ref(), "CreateItems")?;
            if let Some(id) = result.item.as_ref().and_then(crate::builders::extract_item_kiid) {
                kiids.push(id);
            }
        }
        self.confirm_items_exist(&kiids, "CreateItems")?;
        Ok(())
    }

    /// Update existing items by KIID. Generic wrapper mirroring create_items/delete_items;
    /// each `Any` must be a fully-formed board item with an existing `id` populated.
    /// Checks the request-level status, that every requested item got a result, each
    /// item's own update result, and — same as create_items — independently re-resolves
    /// each updated item by KIID afterward rather than trusting the response alone.
    pub fn update_items(&self, items: Vec<prost_types::Any>) -> Result<()> {
        let requested = items.len();
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::UpdateItems {
            header: Some(header),
            items,
        };
        let resp_any =
            self.send_mutating_command(&cmd, "kiapi.common.commands.UpdateItems", "Update items")?
                .ok_or_else(|| anyhow::anyhow!("UpdateItems returned no response body"))?;
        let resp: kiapi::common::commands::UpdateItemsResponse = unpack_any(&resp_any)?;
        check_item_request_status(resp.status, "UpdateItems")?;
        check_result_count(requested, resp.updated_items.len(), "UpdateItems")?;
        let mut kiids = Vec::with_capacity(resp.updated_items.len());
        for result in &resp.updated_items {
            check_item_status(result.status.as_ref(), "UpdateItems")?;
            if let Some(id) = result.item.as_ref().and_then(crate::builders::extract_item_kiid) {
                kiids.push(id);
            }
        }
        self.confirm_items_exist(&kiids, "UpdateItems")?;
        Ok(())
    }

    /// Independently re-resolve items by KIID via GetItemsById after a CreateItems/
    /// UpdateItems call, confirming the mutation actually landed on the live board
    /// rather than trusting the mutating call's own response — the same round-trip
    /// principle `delete_items` already applies (pre-check) and `place_footprint`/
    /// `add_via` already apply (post-check by re-query).
    fn confirm_items_exist(&self, kiids: &[String], op: &str) -> Result<()> {
        if kiids.is_empty() {
            return Ok(());
        }
        let found = self.get_items_by_id(kiids)?;
        if found.len() < kiids.len() {
            anyhow::bail!(
                "{} reported success but only {} of {} item(s) independently re-resolved by \
                 KIID afterward — the mutation did not fully land",
                op,
                found.len(),
                kiids.len()
            );
        }
        Ok(())
    }

    /// Poll `check` until it returns `Ok(true)` or `timeout` elapses. Needed because some
    /// KiCAD API handlers (see `refill_zones`) report success from a synchronous call but
    /// perform the actual mutation later, on a deferred event-loop tick — a same-instant
    /// re-query can't be trusted there, so give the real effect a little time to land
    /// before deciding a round-trip check failed.
    fn poll_until<F>(
        &self,
        timeout: std::time::Duration,
        interval: std::time::Duration,
        mut check: F,
    ) -> Result<bool>
    where
        F: FnMut() -> Result<bool>,
    {
        let start = std::time::Instant::now();
        loop {
            if check()? {
                return Ok(true);
            }
            if start.elapsed() >= timeout {
                return Ok(false);
            }
            std::thread::sleep(interval);
        }
    }

    /// Delete items by KIID.
    ///
    /// Confirms every requested KIID actually exists BEFORE issuing the
    /// delete (via GetItemsById, the one command that reliably resolves an
    /// arbitrary KIID regardless of item type) rather than trusting
    /// DeleteItems' own response to say so. Confirmed live: KiCAD 10's
    /// DeleteItems leaves `deleted_items` empty regardless of whether the
    /// delete succeeded or failed — a real, verified-present KIID and a
    /// bogus one both come back with zero entries and IRS_OK, so neither a
    /// response count check nor a post-delete existence check can tell
    /// them apart (post-delete, a *successful* delete's target is also, by
    /// definition, "not found" — indistinguishable from never having
    /// existed). A pre-delete existence check has no such ambiguity.
    pub fn delete_items(&self, ids: Vec<String>) -> Result<()> {
        let existing = self.get_items_by_id(&ids)?;
        if existing.len() < ids.len() {
            anyhow::bail!(
                "DeleteItems requested {} item(s) but only {} of them exist on the board \
                 — the rest are bogus/nonexistent KIIDs",
                ids.len(),
                existing.len()
            );
        }

        let header = self.make_header()?;
        let cmd = kiapi::common::commands::DeleteItems {
            header: Some(header),
            item_ids: ids
                .iter()
                .map(|id| kiapi::common::types::Kiid { value: id.clone() })
                .collect(),
        };
        let resp_any =
            self.send_mutating_command(&cmd, "kiapi.common.commands.DeleteItems", "Delete items")?
                .ok_or_else(|| anyhow::anyhow!("DeleteItems returned no response body"))?;
        let resp: kiapi::common::commands::DeleteItemsResponse = unpack_any(&resp_any)?;
        check_item_request_status(resp.status, "DeleteItems")?;
        for result in &resp.deleted_items {
            check_deletion_status(result.status, "DeleteItems")?;
        }
        Ok(())
    }

    /// Resolve KIIDs to their current item bodies, regardless of item type.
    /// Used to confirm existence before/after mutating calls whose own
    /// response bodies can't be trusted for that (see `delete_items`).
    fn get_items_by_id(&self, ids: &[String]) -> Result<Vec<prost_types::Any>> {
        if ids.is_empty() {
            return Ok(vec![]);
        }
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::GetItemsById {
            header: Some(header),
            items: ids
                .iter()
                .map(|id| kiapi::common::types::Kiid { value: id.clone() })
                .collect(),
        };
        match self.send_command(&cmd, "kiapi.common.commands.GetItemsById") {
            Ok(Some(any)) => {
                let resp: kiapi::common::commands::GetItemsResponse = unpack_any(&any)?;
                Ok(resp.items)
            }
            Ok(None) => Ok(vec![]),
            Err(e) => {
                // Confirmed live: KiCAD returns a hard AS_BAD_REQUEST (rather
                // than an empty result) when NONE of the requested KIIDs
                // resolve — a legitimate "0 found" answer here, not a
                // transport failure.
                if e.to_string().contains("AS_BAD_REQUEST") {
                    Ok(vec![])
                } else {
                    Err(e)
                }
            }
        }
    }

    /// Refill zones on the board.
    ///
    /// KiCAD's own RefillZones handler (confirmed by reading pcbnew/api/
    /// api_handler_pcb.cpp's `handleRefillZones`) only fills synchronously when there's
    /// no GUI frame attached (true headless CLI mode). Konnect's container runs a real
    /// pcbnew GUI process under Xvfb — not KiCad's true headless mode, which doesn't
    /// exist until KiCad 11 — so `frame()` is always non-null here, which means KiCad
    /// takes the *other* branch: it schedules the fill via `frame()->CallAfter(...)` and
    /// returns success immediately, before the fill geometry is actually computed. The
    /// outer "request accepted" response is therefore not trustworthy proof the fill
    /// happened by the time this call returns — poll each zone's own `filled` flag via
    /// GetItems until every zone reports filled (or time out) before reporting success.
    pub fn refill_zones(&self) -> Result<()> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::RefillZones {
            board: Some(doc),
            zones: vec![],
        };
        self.send_mutating_command(&cmd, "kiapi.board.commands.RefillZones", "Refill zones")?;

        let zones = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbZone)?;
        if zones.is_empty() {
            // Nothing to fill; RunAction(zoneFillAll) is a harmless no-op either way.
            debug!("RefillZones: no zones on board, nothing to poll for");
            return Ok(());
        }
        debug!("RefillZones: polling fill state of {} zone(s)", zones.len());

        let all_filled = self.poll_until(
            std::time::Duration::from_secs(15),
            std::time::Duration::from_millis(200),
            || {
                let zones = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbZone)?;
                let filled_count = zones
                    .iter()
                    .filter(|z| {
                        kiapi::board::types::Zone::decode(z.value.as_slice())
                            .map(|zone| zone.filled)
                            .unwrap_or(false)
                    })
                    .count();
                debug!("RefillZones: {}/{} zone(s) filled", filled_count, zones.len());
                Ok(filled_count == zones.len())
            },
        )?;

        if !all_filled {
            anyhow::bail!(
                "RefillZones was accepted by KiCAD but the fill did not complete within 15s \
                 of polling — KiCAD defers the actual fill to a later event-loop tick when \
                 running with a GUI frame, and it never reported all zones filled"
            );
        }
        Ok(())
    }

    /// Save the open board document.
    ///
    /// This fires KiCAD's SaveDocument command and returns as soon as KiCAD
    /// *acks* the command — it does not confirm the on-disk write itself
    /// has actually landed by the time this returns (the same
    /// async-completion bug class `refill_zones` documents and works
    /// around above). Callers that need the saved content back should
    /// prefer `save_board_to_string()`, which round-trips the content over
    /// IPC directly rather than trusting a disk write and then re-reading
    /// it.
    pub fn save_board(&self) -> Result<()> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::common::commands::SaveDocument {
            document: Some(doc),
        };
        self.send_command(&cmd, "kiapi.common.commands.SaveDocument")?;
        Ok(())
    }

    /// Save the open board document to an in-memory string, without
    /// touching disk, via KiCAD's SaveDocumentToString IPC command. Where
    /// implemented, this is what `save_splice_revert` prefers over
    /// `save_board()`'s write-then-reread-from-disk path, since it avoids
    /// trusting an unverified disk write (see `save_board`'s own caveat).
    /// Bails with a clear error if KiCAD returns empty contents (observed
    /// when this command isn't implemented on a given KiCAD version), so
    /// callers can fall back to the disk-based path.
    pub fn save_board_to_string(&self) -> Result<String> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::common::commands::SaveDocumentToString {
            document: Some(doc),
        };
        let response_any = self.send_command(&cmd, "kiapi.common.commands.SaveDocumentToString")?;
        let contents = match response_any {
            Some(any) => {
                let resp: kiapi::common::commands::SavedDocumentResponse = unpack_any(&any)?;
                resp.contents
            }
            None => String::new(),
        };
        if contents.is_empty() {
            anyhow::bail!(
                "SaveDocumentToString returned empty contents — likely unimplemented on this \
                 KiCAD version"
            );
        }
        Ok(contents)
    }

    /// Resolve the on-disk path of the currently open board, via the same
    /// DocumentSpecifier (from GetOpenDocuments) that SaveDocument/
    /// RevertDocument use internally to identify it. `board_filename` is
    /// just the bare filename (confirmed live — e.g. "herc-verify.kicad_pcb",
    /// not a path, despite reading like one), so it has to be joined with
    /// the project directory from the specifier's `project.path`.
    fn board_file_path(&self) -> Result<std::path::PathBuf> {
        let doc = self.get_board_document()?;
        let filename = match &doc.identifier {
            Some(kiapi::common::types::document_specifier::Identifier::BoardFilename(name)) => {
                name.clone()
            }
            _ => anyhow::bail!("open board document has no board_filename"),
        };
        let dir = doc
            .project
            .as_ref()
            .map(|p| p.path.as_str())
            .filter(|p| !p.is_empty())
            .ok_or_else(|| anyhow::anyhow!("open board document has no project path"))?;
        Ok(std::path::PathBuf::from(dir).join(filename))
    }

    /// Confirm `requested` is the same file KiCAD currently has open, before a
    /// caller-supplied `board` argument is trusted to justify a mutating IPC
    /// call.
    ///
    /// Root cause of the 2026-08-29 "place_component reports success and
    /// get_component_list shows the new part, but the saved .kicad_pcb has
    /// zero footprints" report: KiCAD's IPC API drives a single running GUI
    /// instance with exactly one document open at a time (the container's
    /// entrypoint.sh picks one board via a one-time directory scan at boot --
    /// there is no IPC command to switch documents afterward). Every mutating
    /// tool handler declared a `board` argument in its schema but never
    /// checked it against reality, so a call whose `board` didn't match
    /// KiCAD's one open document silently mutated whatever WAS open while
    /// still reporting success -- the caller's actual target file was never
    /// touched. This is confirmed live: a `place_component` call against a
    /// project that was never the one open in KiCad succeeds, is visible via
    /// `get_component_list` (same open board), and survives `save_project`
    /// (same open board saved back to itself) -- while the requested board's
    /// own .kicad_pcb on disk is untouched throughout.
    ///
    /// This is a distinct, previously-unfixed gap from the Phase 2
    /// (2026-08-04/05) begin_commit/end_commit + splice/RevertDocument work,
    /// which is confirmed still correct for the case that was actually
    /// tested (requested board == the one open board).
    pub fn verify_board_matches(&self, requested: &std::path::Path) -> Result<()> {
        let open_path = self.board_file_path()?;
        let requested_canon = requested
            .canonicalize()
            .unwrap_or_else(|_| requested.to_path_buf());
        let open_canon = open_path
            .canonicalize()
            .unwrap_or_else(|_| open_path.clone());
        if requested_canon != open_canon {
            anyhow::bail!(
                "requested board '{}' is not the board KiCAD currently has open \
                 ('{}'). KiCAD's IPC API drives a single open document -- this call \
                 would otherwise silently mutate the open board instead of the \
                 requested file. Open '{}' in the KiCAD GUI (or restart the konnect \
                 container with only that project present under /work) before \
                 retrying.",
                requested.display(),
                open_path.display(),
                requested.display()
            );
        }
        Ok(())
    }

    /// Reload the open board from disk, discarding pcbnew's in-memory state.
    /// Confirmed (by reading KiCAD's own source) to do
    /// `SetContentModified(false)` -> `ReleaseFile()` -> `OpenProjectFiles(...,
    /// KICTL_REVERT)`, with no confirmation dialog even under Xvfb.
    pub fn revert_document(&self) -> Result<()> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::common::commands::RevertDocument {
            document: Some(doc),
        };
        self.send_command(&cmd, "kiapi.common.commands.RevertDocument")?;
        Ok(())
    }

    /// Shared save→splice→revert sequence used by `add_via` and
    /// `place_footprint` to route new items around KiCAD 10's dead
    /// `ParseAndCreateItemsFromString` stub (see either caller's doc
    /// comment for the full background).
    ///
    /// Serialized by `SAVE_SPLICE_LOCK` for the whole sequence: two
    /// overlapping calls (e.g. one from `add_via`, one from
    /// `place_footprint`) would otherwise race on the same board file via
    /// a non-atomic read-modify-write.
    ///
    /// Prefers `save_board_to_string()` (an in-memory save with no
    /// unverified disk round-trip) over `save_board()` + re-reading the
    /// file from disk; if `SaveDocumentToString` isn't implemented on this
    /// KiCAD version (signaled by `save_board_to_string()`'s "empty
    /// contents" error), falls back to the disk-based path — but that
    /// fallback still runs its content through the same validated
    /// `splice_sexp_into_string`, never the old unchecked
    /// file-splicing helper.
    ///
    /// Takes a pre-splice backup of the board file first; if anything in
    /// the sequence fails — including `revert_document()` itself failing
    /// or timing out — restores those bytes (best-effort) before
    /// propagating the original error, so a half-applied splice is never
    /// left on disk for some later unrelated save to persist back over.
    fn save_splice_revert(&self, sexp_block: &str) -> Result<()> {
        let _guard = SAVE_SPLICE_LOCK
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());

        let board_path = self.board_file_path()?;
        let backup = std::fs::read(&board_path).ok();

        let result = (|| -> Result<()> {
            let spliced = match self.save_board_to_string() {
                Ok(contents) => splice_sexp_into_string(&contents, sexp_block)?,
                Err(e) if e.to_string().contains("empty contents") => {
                    // SaveDocumentToString isn't implemented on this KiCAD
                    // version — fall back to the old disk-based save/read,
                    // but still go through the validated splice helper.
                    self.save_board()?;
                    let contents = std::fs::read_to_string(&board_path).with_context(|| {
                        format!("could not read board file {}", board_path.display())
                    })?;
                    splice_sexp_into_string(&contents, sexp_block)?
                }
                Err(e) => return Err(e),
            };
            std::fs::write(&board_path, spliced)
                .with_context(|| format!("could not write board file {}", board_path.display()))?;
            self.revert_document()
        })();

        if result.is_err() {
            if let Some(bytes) = backup {
                let _ = std::fs::write(&board_path, bytes);
            }
        }

        result
    }

    /// Begin a commit (undo group).
    pub fn begin_commit(&self) -> Result<String> {
        let cmd = kiapi::common::commands::BeginCommit {};
        let response_any = self.send_command(&cmd, "kiapi.common.commands.BeginCommit")?;
        if let Some(any) = response_any {
            let resp: kiapi::common::commands::BeginCommitResponse = unpack_any(&any)?;
            Ok(resp.id.map(|id| id.value).unwrap_or_default())
        } else {
            Ok(String::new())
        }
    }

    /// End a commit (push or drop).
    pub fn end_commit(
        &self,
        commit_id: &str,
        action: kiapi::common::commands::CommitAction,
        message: &str,
    ) -> Result<()> {
        let cmd = kiapi::common::commands::EndCommit {
            id: Some(kiapi::common::types::Kiid {
                value: commit_id.to_string(),
            }),
            action: action as i32,
            message: message.to_string(),
        };
        self.send_command(&cmd, "kiapi.common.commands.EndCommit")?;
        Ok(())
    }

    /// Push (commit) changes.
    pub fn push_commit(&self, commit_id: &str, description: &str) -> Result<()> {
        self.end_commit(
            commit_id,
            kiapi::common::commands::CommitAction::CmaCommit,
            description,
        )
    }

    /// Drop (rollback) changes.
    pub fn drop_commit(&self, commit_id: &str) -> Result<()> {
        self.end_commit(
            commit_id,
            kiapi::common::commands::CommitAction::CmaDrop,
            "",
        )
    }

    // ─── PCB Item Operations (real protobuf implementations) ───────────

    /// Resolve a net name to its net code by querying GetNets.
    pub fn resolve_net_code(&self, net_name: &str) -> Result<i32> {
        let nets = self.get_nets()?;
        nets.iter()
            .find(|n| n.name == net_name)
            .map(|n| n.netcode)
            .ok_or_else(|| anyhow::anyhow!("Net '{}' not found on board", net_name))
    }

    /// Find a footprint by reference and return its IpcFootprint + KIID.
    pub fn get_footprint(&self, reference: &str) -> Result<Option<IpcFootprint>> {
        let footprints = self.list_footprints()?;
        Ok(footprints.into_iter().find(|fp| fp.reference == reference))
    }

    /// Find a footprint's KIID by reference.
    fn find_footprint_kiid(&self, reference: &str) -> Result<String> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(fp) = kiapi::board::types::FootprintInstance::decode(item.value.as_slice()) {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    if let Some(id) = &fp.id {
                        return Ok(id.value.clone());
                    }
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found on board", reference)
    }

    /// Add a track segment to the board.
    #[allow(clippy::too_many_arguments)]
    pub fn add_track(
        &self,
        net_name: &str,
        layer: &str,
        width: f64,
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    ) -> Result<()> {
        let net_code = self.resolve_net_code(net_name)?;
        let track = crate::builders::build_track(net_name, net_code, layer, width, x1, y1, x2, y2);
        let any = crate::builders::pack_any(&track, "kiapi.board.types.Track");
        self.create_items(vec![any])?;
        Ok(())
    }

    /// Add a via to the board via S-expression string (simpler than full protobuf
    /// PadStack construction). KiCAD 10's ParseAndCreateItemsFromString is a dead,
    /// unimplemented no-op stub (confirmed by reading pcbnew/api/api_handler_pcb.cpp)
    /// — it validates and returns success without ever creating anything. Route
    /// around it via `save_splice_revert()`: save the board (in-memory when
    /// possible), splice the via S-expression in as a top-level sibling, write it
    /// back, then have KiCAD reload. Returns the new via's real KIID (found by
    /// matching position + net after reload)
    /// rather than echoing the caller's input args back as if they were confirmed.
    /// Also decodes the re-queried via's PadStack (drill.diameter + each
    /// copper_layers[].size) and confirms it matches the requested drill/pad
    /// size — position+net alone previously let a wrong-geometry via (e.g.
    /// KiCAD silently falling back to a default drill/pad) report success.
    pub fn add_via(&self, net_name: &str, x: f64, y: f64, drill: f64, pad_size: f64) -> Result<String> {
        let net_code = self.resolve_net_code(net_name)?;
        let sexp = crate::builders::via_sexp(net_name, net_code, x, y, drill, pad_size);

        self.save_splice_revert(&sexp)?;

        let x_nm = crate::builders::mm_to_nm(x);
        let y_nm = crate::builders::mm_to_nm(y);
        let drill_nm = crate::builders::mm_to_nm(drill);
        let size_nm = crate::builders::mm_to_nm(pad_size);
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbVia)?;
        for item in &items {
            if let Ok(via) = kiapi::board::types::Via::decode(item.value.as_slice()) {
                let pos = via.position.unwrap_or_default();
                // KiCAD omits the NetCode message entirely (leaves it None,
                // not Some(0)) for net 0 — confirmed live. 0 is also net 0's
                // fallback everywhere else in this file (see get_nets()'s
                // IpcNet construction above), so match that convention here.
                let via_net_code = via
                    .net
                    .as_ref()
                    .and_then(|n| n.code.as_ref())
                    .map(|c| c.value)
                    .unwrap_or(0);
                if pos.x_nm == x_nm && pos.y_nm == y_nm && via_net_code == net_code {
                    // Position and net matched, but KiCAD has separately
                    // reported an "accepted" mutation with the wrong result
                    // before (the whole reason every mutating tool in this
                    // fork round-trips) — decode the padstack it actually
                    // persisted and confirm the drill/pad geometry too,
                    // not just where the via sits and what net it's on.
                    let pad_stack = via.pad_stack.as_ref().ok_or_else(|| {
                        anyhow::anyhow!(
                            "via at ({}, {}) on net '{}' matched position/net but has no \
                             pad_stack in the re-queried item — can't confirm drill/pad size",
                            x,
                            y,
                            net_name
                        )
                    })?;
                    let actual_drill_nm = pad_stack
                        .drill
                        .as_ref()
                        .and_then(|d| d.diameter.as_ref())
                        .map(|d| d.x_nm)
                        .ok_or_else(|| {
                            anyhow::anyhow!(
                                "via at ({}, {}) on net '{}' has no drill diameter in its \
                                 re-queried pad_stack — can't confirm it matches the requested \
                                 {}mm",
                                x,
                                y,
                                net_name,
                                drill
                            )
                        })?;
                    if actual_drill_nm != drill_nm {
                        anyhow::bail!(
                            "via at ({}, {}) on net '{}' was created with drill {}mm, not the \
                             requested {}mm",
                            x,
                            y,
                            net_name,
                            crate::builders::nm_to_mm(actual_drill_nm),
                            drill
                        );
                    }
                    if pad_stack.copper_layers.is_empty() {
                        anyhow::bail!(
                            "via at ({}, {}) on net '{}' has no copper_layers in its re-queried \
                             pad_stack — can't confirm pad size matches the requested {}mm",
                            x,
                            y,
                            net_name,
                            pad_size
                        );
                    }
                    for layer in &pad_stack.copper_layers {
                        let size = layer.size.as_ref().ok_or_else(|| {
                            anyhow::anyhow!(
                                "via at ({}, {}) on net '{}' has a copper_layers entry with no \
                                 size — can't confirm pad size matches the requested {}mm",
                                x,
                                y,
                                net_name,
                                pad_size
                            )
                        })?;
                        if size.x_nm != size_nm || size.y_nm != size_nm {
                            anyhow::bail!(
                                "via at ({}, {}) on net '{}' was created with pad size \
                                 {}x{}mm, not the requested {}mm",
                                x,
                                y,
                                net_name,
                                crate::builders::nm_to_mm(size.x_nm),
                                crate::builders::nm_to_mm(size.y_nm),
                                pad_size
                            );
                        }
                    }
                    return Ok(via.id.map(|k| k.value).unwrap_or_default());
                }
            }
        }
        anyhow::bail!(
            "placed via at ({}, {}) on net '{}' but it's not visible on the board after reload",
            x,
            y,
            net_name
        )
    }

    /// Delete a track by UUID.
    pub fn delete_track(&self, uuid: &str) -> Result<()> {
        self.delete_items(vec![uuid.to_string()])
    }

    /// Query tracks, optionally filtered by net and/or layer.
    pub fn get_tracks(
        &self,
        net_filter: Option<&str>,
        layer_filter: Option<&str>,
    ) -> Result<Vec<IpcTrack>> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbTrace)?;
        let mut tracks = Vec::new();
        for item in &items {
            if let Ok(track) = kiapi::board::types::Track::decode(item.value.as_slice()) {
                let net_name = track.net.as_ref().map(|n| n.name.as_str()).unwrap_or("");
                let layer_name = layer_enum_to_name(track.layer);

                // Apply net filter
                if let Some(nf) = net_filter {
                    if net_name != nf {
                        continue;
                    }
                }
                // Apply layer filter
                if let Some(lf) = layer_filter {
                    if layer_name != lf {
                        continue;
                    }
                }

                let start = track.start.as_ref();
                let end = track.end.as_ref();
                tracks.push(IpcTrack {
                    net_name: net_name.to_string(),
                    layer: layer_name.to_string(),
                    width: track
                        .width
                        .as_ref()
                        .map(|w| crate::builders::nm_to_mm(w.value_nm))
                        .unwrap_or(0.25),
                    start: IpcVector2 {
                        x: start
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: start
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                    end: IpcVector2 {
                        x: end
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: end
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                });
            }
        }
        Ok(tracks)
    }

    /// Move a footprint to a new position. Returns the independently re-queried,
    /// post-move footprint state (not the caller's input) — see the round-trip
    /// check below.
    pub fn move_footprint(&self, reference: &str, x: f64, y: f64) -> Result<IpcFootprint> {
        // Find the footprint, update position, send UpdateItems
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(mut fp) =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    let old = fp.position.unwrap_or_default();
                    let new_pos = crate::builders::vec2(x, y);
                    fp.position = Some(new_pos);
                    // KiCAD carries the footprint's children (pads, silk,
                    // text) in absolute board coordinates and re-creates them
                    // verbatim on update, so they must be shifted along with
                    // the anchor (issue #23).
                    crate::transform::transform_footprint_children(
                        &mut fp,
                        &crate::transform::Xform::Translate {
                            dx_nm: new_pos.x_nm - old.x_nm,
                            dy_nm: new_pos.y_nm - old.y_nm,
                        },
                    )?;
                    let any = crate::builders::pack_any(&fp, "kiapi.board.types.FootprintInstance");

                    // Route through update_items() rather than calling
                    // send_mutating_command inline (as this used to) so moves get the
                    // same per-item status + independent re-resolve-by-KIID checks
                    // every other update goes through, instead of only trusting the
                    // outer "request accepted" response.
                    self.update_items(vec![any])?;

                    // Round-trip: update_items() only proves *an* item update landed,
                    // not that THIS footprint ended up at THIS position — re-fetch by
                    // reference and confirm the reported position actually matches
                    // what was requested.
                    let confirmed = self.get_footprint(reference)?.ok_or_else(|| {
                        anyhow::anyhow!(
                            "moved '{}' but it's no longer resolvable by reference afterward",
                            reference
                        )
                    })?;
                    const EPS_MM: f64 = 1e-3;
                    if (confirmed.position.x - x).abs() > EPS_MM
                        || (confirmed.position.y - y).abs() > EPS_MM
                    {
                        anyhow::bail!(
                            "move_footprint reported success for '{}' but re-query shows it \
                             at ({}, {}), not the requested ({}, {})",
                            reference,
                            confirmed.position.x,
                            confirmed.position.y,
                            x,
                            y
                        );
                    }
                    return Ok(confirmed);
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found", reference)
    }

    /// Rotate a footprint to a new angle. Returns the independently re-queried,
    /// post-rotate footprint state (not the caller's input) — see the round-trip
    /// check below.
    pub fn rotate_footprint(&self, reference: &str, angle: f64) -> Result<IpcFootprint> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(mut fp) =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    let old_deg = fp
                        .orientation
                        .as_ref()
                        .map(|a| a.value_degrees)
                        .unwrap_or(0.0);
                    fp.orientation = Some(kiapi::common::types::Angle {
                        value_degrees: angle,
                    });
                    // Children are carried in absolute board coordinates and
                    // angles; rotate them around the anchor like KiCAD's
                    // FOOTPRINT::SetOrientation does natively (issue #23).
                    let anchor = fp.position.unwrap_or_default();
                    crate::transform::transform_footprint_children(
                        &mut fp,
                        &crate::transform::Xform::Rotate {
                            cx_nm: anchor.x_nm,
                            cy_nm: anchor.y_nm,
                            delta_deg: angle - old_deg,
                        },
                    )?;
                    let any = crate::builders::pack_any(&fp, "kiapi.board.types.FootprintInstance");

                    // Same fix as move_footprint: go through update_items() for the
                    // shared validation + re-resolve-by-KIID check instead of an
                    // unvalidated inline send_mutating_command.
                    self.update_items(vec![any])?;

                    // Round-trip: re-fetch by reference and confirm the reported angle
                    // actually matches what was requested (mod 360, since KiCAD
                    // normalizes orientation).
                    let confirmed = self.get_footprint(reference)?.ok_or_else(|| {
                        anyhow::anyhow!(
                            "rotated '{}' but it's no longer resolvable by reference afterward",
                            reference
                        )
                    })?;
                    const EPS_DEG: f64 = 1e-3;
                    let norm = |a: f64| ((a % 360.0) + 360.0) % 360.0;
                    if (norm(confirmed.rotation) - norm(angle)).abs() > EPS_DEG {
                        anyhow::bail!(
                            "rotate_footprint reported success for '{}' but re-query shows \
                             rotation {}, not the requested {}",
                            reference,
                            confirmed.rotation,
                            angle
                        );
                    }
                    return Ok(confirmed);
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found", reference)
    }

    /// Delete a footprint by reference.
    pub fn delete_footprint(&self, reference: &str) -> Result<()> {
        let kiid = self.find_footprint_kiid(reference)?;
        self.delete_items(vec![kiid])
    }

    /// Place a footprint by reading its real definition from the KiCAD footprint
    /// library on disk and splicing it directly into the board file.
    pub fn place_footprint(
        &self,
        lib_id: &str,
        x: f64,
        y: f64,
        rotation: f64,
        layer: &str,
        reference: &str,
    ) -> Result<IpcFootprint> {
        // KiCAD 10 IPC doesn't have a direct "place footprint from library" command,
        // so the real definition is read off disk and spliced in with
        // position/layer/reference (see instantiate_footprint_sexp) as before.
        //
        // What changed: KiCAD 10.0's ParseAndCreateItemsFromString is a dead,
        // unimplemented no-op stub (confirmed by reading
        // pcbnew/api/api_handler_pcb.cpp) — it validates the request and returns a
        // clean success WITHOUT ever reading the S-expression content or creating
        // anything. Routing the correct S-expression through it can never work.
        // Instead: save the live board to disk, splice the footprint S-expression
        // in as a top-level sibling, write it back, then have KiCAD reload the file
        // from disk so the new footprint actually lands in its in-memory model.
        let fp_path = crate::builders::resolve_footprint_file(lib_id)?;
        let raw = std::fs::read_to_string(&fp_path)
            .with_context(|| format!("could not read footprint file {}", fp_path.display()))?;
        let sexp = crate::builders::instantiate_footprint_sexp(
            &raw,
            crate::builders::mm_to_nm(x) as f64 / 1_000_000.0,
            crate::builders::mm_to_nm(y) as f64 / 1_000_000.0,
            rotation,
            layer,
            reference,
        )?;

        self.save_splice_revert(&sexp)?;

        // Read back the observed state rather than echoing the caller's input
        // args — reloading and finding it is what actually proves the footprint
        // made it onto the board (the previous echo-back is exactly why placement
        // looked like it worked when it didn't: nothing was ever verified).
        self.get_footprint(reference)?.ok_or_else(|| {
            anyhow::anyhow!(
                "placed '{}' but it's not visible on the board after reload",
                reference
            )
        })
    }

    /// Get board extents (bounding box of all items).
    pub fn get_board_extents(&self) -> Result<IpcBoardExtents> {
        // Use GetBoundingBox with no specific items = board extents
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::GetBoundingBox {
            header: Some(header),
            items: vec![], // empty = all items
            mode: kiapi::common::commands::BoundingBoxMode::BbmItemOnly as i32,
        };
        let resp_any = self.send_command(&cmd, "kiapi.common.commands.GetBoundingBox")?;
        if let Some(any) = resp_any {
            let resp: kiapi::common::commands::GetBoundingBoxResponse = unpack_any(&any)?;
            if let Some(bbox) = resp.boxes.first() {
                let pos = bbox.position.as_ref();
                let size = bbox.size.as_ref();
                return Ok(IpcBoardExtents {
                    min: IpcVector2 {
                        x: pos
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: pos
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                    max: IpcVector2 {
                        x: pos
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0)
                            + size
                                .map(|s| crate::builders::nm_to_mm(s.x_nm))
                                .unwrap_or(0.0),
                        y: pos
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0)
                            + size
                                .map(|s| crate::builders::nm_to_mm(s.y_nm))
                                .unwrap_or(0.0),
                    },
                });
            }
        }
        anyhow::bail!("No bounding box returned from KiCAD")
    }

    /// Get enabled layers.
    pub fn get_layers(&self) -> Result<Vec<IpcLayer>> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::GetBoardEnabledLayers { board: Some(doc) };
        let resp_any = self.send_command(&cmd, "kiapi.board.commands.GetBoardEnabledLayers")?;
        if let Some(any) = resp_any {
            let resp: kiapi::board::commands::BoardEnabledLayersResponse = unpack_any(&any)?;
            let layers = resp
                .layers
                .iter()
                .map(|&l| {
                    let bl = kiapi::board::types::BoardLayer::try_from(l)
                        .unwrap_or(kiapi::board::types::BoardLayer::BlUndefined);
                    IpcLayer {
                        name: bl
                            .as_str_name()
                            .trim_start_matches("BL_")
                            .replace('_', ".")
                            .to_string(),
                        id: l,
                        kind: String::new(),
                    }
                })
                .collect();
            Ok(layers)
        } else {
            Ok(vec![])
        }
    }

    /// Run an arbitrary tool action in KiCAD (e.g. to trigger a refresh).
    pub fn run_action(&self, action: &str) -> Result<()> {
        let cmd = kiapi::common::commands::RunAction {
            action: action.to_string(),
        };
        self.send_command(&cmd, "kiapi.common.commands.RunAction")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splice_sexp_into_string_rejects_non_kicad_pcb_input() {
        let err = splice_sexp_into_string("(footprint \"foo\")", "(via)")
            .expect_err("non-kicad_pcb input must not splice");
        assert!(
            err.to_string()
                .contains("do not parse as a valid .kicad_pcb"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn splice_sexp_into_string_rejects_truncated_input() {
        // Missing closing paren — a torn/partial read of the board file.
        let err = splice_sexp_into_string("(kicad_pcb (version 20240101)", "(via)")
            .expect_err("truncated input must not splice");
        assert!(
            err.to_string()
                .contains("do not parse as a valid .kicad_pcb"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn splice_sexp_into_string_inserts_block_before_final_paren() {
        let out = splice_sexp_into_string("(kicad_pcb (version 20240101))", "(via (at 1 2))")
            .expect("valid kicad_pcb input should splice cleanly");
        assert!(out.contains("(via (at 1 2))"));
        // The result must itself still parse as a well-formed kicad_pcb
        // document (checked by splice_sexp_into_string itself before
        // returning Ok) — i.e. the block landed inside the outer list,
        // not appended after its closing paren.
        let reparsed = konnect_sexp::parse_sexp(&out).expect("spliced output must reparse");
        assert_eq!(reparsed.head(), Some("kicad_pcb"));
        assert!(reparsed.find("via").is_some());
    }
}
