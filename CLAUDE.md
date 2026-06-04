# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Build
cargo build

# Build release
cargo build --release

# Run
cargo run

# Check (fast, no codegen)
cargo check

# Lint
cargo clippy

# Run with logging
RUST_LOG=info cargo run
```

The dev environment uses Nix (`shell.nix`) with a nightly Rust toolchain and SDL2 libraries; on non-Nix systems SDL2, SDL2_mixer, SDL2_gfx, and SDL2_ttf must be installed separately.

## Architecture

This is a Rust game demo using the **Legion ECS** framework with a fixed set of systems executed in order each frame. The entry point is `src/main.rs`, which owns the SDL2 window, the game loop, and all resource initialization.

### Coordinate conventions

- Voxlap world space is **z-down**: small z = up, large z = down
- Body-local axes: **−Z = forward (nose)**, **+X = right**, **+Y = up**
- The `Camera` struct voxlap expects uses `forward`, `right`, and `down` (not `up`); the render system negates the body's `+Y` to produce `down`
- `VSID = 32` sets the voxel world grid dimension (32×32 columns)