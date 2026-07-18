# Context for Claude Code

This file exists to carry context across machines/sessions. See [README.md](README.md)
for the project's purpose and scope — read that first.

## Status as of 2026-07-18

The Cargo project is scaffolded but **`cargo build` has never successfully
completed**. Nothing about the code is known to be broken — the scaffold
just hasn't been verified yet because the machine that created it hit
environment problems unrelated to the code. Read this before assuming the
crate compiles.

### What's in place
- [Cargo.toml](Cargo.toml) — `geomap-engine` lib crate, `prost` + `prost-types`
  deps, `prost-build` as a build-dependency.
- [build.rs](build.rs) — compiles `proto/frame.proto` via `prost-build`.
- [proto/frame.proto](proto/frame.proto) — **this is the real, final schema**
  (not a placeholder), provided by the project owner: `CameraPose`,
  `CameraIntrinsics`, `Detection`, `Frame`, `MapObject`, `SceneMap`, all in
  `package geomap`.
- [src/lib.rs](src/lib.rs) — includes the generated `proto` module
  (`OUT_DIR/geomap.rs`) and a minimal `Engine` with
  `ingest_frame(Frame) -> &SceneMap`. **This is a stub** — it does not yet
  implement projection, association, fusion, or map maintenance (see
  README's "Core responsibilities"). That's the actual next work item once
  the build is verified.

### Why the build was never verified (prior machine, Windows)
`cargo build` failed at the linking stage, unrelated to the Rust code:

1. The only `link.exe` on PATH was a leftover Visual Studio 6.0 (1998)
   linker (`...\VC98\bin\LINK.EXE`), which can't link modern object files
   (`LNK1136: invalid or corrupt file`). Those stale VC98 entries have
   since been **removed from the User PATH** on that machine.
2. Tried switching to the `x86_64-pc-windows-gnu` target to sidestep MSVC
   entirely — got further (past linking) but failed needing `dlltool.exe`
   from MinGW binutils, which rustc's self-contained linker doesn't bundle.
3. That machine's network only allowlisted a few package-registry hosts
   (`crates.io`, `static.rust-lang.org`); `github.com`, `aka.ms`, and
   SourceForge mirror hosts were all unreachable, so neither MinGW-w64 nor
   the normal VS Build Tools bootstrapper (via `aka.ms`) could be fetched.
   A direct (non-`aka.ms`) `download.visualstudio.microsoft.com` URL for
   `vs_BuildTools.exe` was found and was reachable, and an install of the
   `Microsoft.VisualStudio.Workload.VCTools` workload was started — but it
   was stopped mid-download at the user's request before completing (work
   was moving to a different machine instead).

**On a fresh machine**, none of this may apply — just try `cargo build`
first. If it fails on linking with a similar "corrupt file" or "linker not
found" error on Windows, install the Visual Studio Build Tools with the
"Desktop development with C++" workload (or `winget install
Microsoft.VisualStudio.2022.BuildTools` with that workload added), or use
WSL2 / Linux / macOS instead where a system linker (`cc`/`ld`) is normally
already present.

### Git remote
`origin` is set to `git@github.com:ghimirebibek/geomap-engine.git`, but
**this repo was never pushed** from the prior machine — its local SSH key
wasn't authorized on the `ghimirebibek` GitHub account (`Permission denied
(publickey)`), and the user chose to push later from a machine where GitHub
auth already works, rather than fix SSH keys mid-session. Check `git log`
and `git status` before assuming this remote is in sync with GitHub.
