//! Startup capability probe (`konnect --selftest`).
//!
//! On container boot, entrypoint.sh runs this against whatever board pcbnew
//! just opened, exercising the mutating-tool call paths already known to be
//! flaky or KiCAD-API-version-dependent — today that's the save->splice->
//! reload footprint path (`place_footprint`, which depends on KiCAD's
//! on-disk .kicad_mod / .kicad_pcb S-expression format matching what Konnect
//! emits) and via net-clause serialization (`add_via`, which once emitted an
//! invalid `(net 0 "")` clause that KiCAD's parser rejected outright and hung
//! RevertDocument behind an unrecoverable dialog) — and logs a clear
//! PASS/FAIL per capability to stdout, which entrypoint.sh's caller captures
//! in container logs same as everything else.
//!
//! Deliberately narrow and non-fatal: this is meant to catch "this KiCAD
//! version broke us" at boot, not to replace real testing or to block
//! container startup on a probe failure (the whole point is visibility).
//!
//! Runs against the real board that's about to be served, not a separate
//! scratch project (standing up a second pcbnew instance just for this would
//! add real startup complexity/risk for a lightweight check) — so each probe
//! places a uniquely-named/positioned throwaway item at an out-of-the-way
//! location and deletes it again before returning, best-effort, on both the
//! pass and fail paths.

use konnect_ipc::client::KiCadIpcClient;

const PROBE_X: f64 = -500.0;
const PROBE_Y: f64 = -500.0;
const PROBE_REF: &str = "KONNECT_SELFTEST_FP";

/// Run every probe and print PASS/FAIL for each. Returns a process exit code
/// (0 if everything passed, 1 if anything failed) for entrypoint.sh to log
/// against, but callers should treat that as informational, not fatal.
pub fn run_selftest() -> i32 {
    println!("[selftest] Konnect capability probe starting");

    let client = KiCadIpcClient::new("");
    let mut total = 0;
    let mut failures = 0;

    total += 1;
    match probe_place_footprint(&client) {
        Ok(()) => println!("[selftest] PASS place_footprint (save->splice->reload path)"),
        Err(e) => {
            failures += 1;
            println!(
                "[selftest] FAIL place_footprint (save->splice->reload path): {}",
                e
            );
        }
    }

    total += 1;
    match probe_add_via(&client) {
        Ok(()) => println!("[selftest] PASS add_via (net-clause serialization)"),
        Err(e) => {
            failures += 1;
            println!("[selftest] FAIL add_via (net-clause serialization): {}", e);
        }
    }

    println!(
        "[selftest] {}/{} capabilities passed",
        total - failures,
        total
    );

    if failures > 0 {
        1
    } else {
        0
    }
}

fn probe_place_footprint(client: &KiCadIpcClient) -> anyhow::Result<()> {
    // Clean up any leftover from a previous crashed/killed probe run first,
    // so it can't make this run spuriously fail on a duplicate reference.
    let _ = client.delete_footprint(PROBE_REF);

    let placed = client.place_footprint(
        "Resistor_SMD:R_0603_1608Metric",
        PROBE_X,
        PROBE_Y,
        0.0,
        "F.Cu",
        PROBE_REF,
    );

    // Best-effort cleanup regardless of outcome, then report the original result.
    let _ = client.delete_footprint(PROBE_REF);

    let placed = placed?;
    if placed.reference != PROBE_REF {
        anyhow::bail!(
            "placed footprint reference '{}' doesn't match requested '{}'",
            placed.reference,
            PROBE_REF
        );
    }
    Ok(())
}

fn probe_add_via(client: &KiCadIpcClient) -> anyhow::Result<()> {
    // add_via() already independently re-confirms the via lands at the
    // requested position/net before returning its KIID (see
    // konnect-ipc/src/client.rs) — a successful, non-empty KIID here IS the
    // pass signal for the whole round-trip, not just "IPC accepted it".
    let result = client.add_via("", PROBE_X + 5.0, PROBE_Y + 5.0, 0.4, 0.8);

    if let Ok(ref kiid) = result {
        if !kiid.is_empty() {
            let _ = client.delete_items(vec![kiid.clone()]);
        }
    }

    let kiid = result?;
    if kiid.is_empty() {
        anyhow::bail!("add_via returned an empty KIID");
    }
    Ok(())
}
