# Sable on Sunrise

A work-in-progress personal Rust port of [Sable](https://github.com/ryanhcode/sable)
for [Sunrise](https://github.com/sunrise-org/sunrise). This source snapshot comes
from local revision `c592b82bf160`.

The [open-drop demo](https://discord.com/channels/1569116592132857946/1571601717043142696/threads/1573809893758992394)
runs on an experimental Sunrise build with live-world projection support.
There is no installable release here yet. Running the full demo requires a
compatible Sunrise build and SDK.

- `sable-physics` contains the adapted native solver.
- `sable-core` contains sublevel assembly, ownership, splitting and persistence.
- `sable-physics-ffi` and `sable-bridge` connect the two Rust toolchains through
  a versioned C ABI.
- `adapters/sunrise-mod` contains the experimental Sunrise integration.

## Build the Rust workspace

Use the original solver's Rust nightly for the physics crates.

```sh
cargo +nightly-2026-01-29 test --locked --workspace -- --test-threads=1
```

The solver tests run serially because they share process-global native state.
The ABI client can also be built independently of the solver.

```sh
cargo check --locked -p sable-bridge --all-targets
```

## Code and licenses

Sable-derived files retain the [PolyForm Shield 1.0.0 license](crates/sable-physics/LICENSE-SABLE.md).
The [physics](crates/sable-physics/provenance/origin.json),
[core](crates/sable-core/provenance/origin.json), and
[Sunrise adapter](adapters/sunrise-mod/provenance/origin.json) ledgers identify
original sources and retained notices. Companion, JOML, Rapier, and Sunrise
license texts are included alongside the relevant code.
