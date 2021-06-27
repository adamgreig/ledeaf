# LEDEAF Firmware

Firmware for LEDEAF. Written in Rust.

## Build Requirements

* You must have a working Rust compiler installed. Visit
[rustup.rs](https://rustup.rs) to install Rust.

* Your Rust toolchain needs the `thumbv7em-none-eabihf` target installed:

```
rustup target add thumbv7em-none-eabihf
```

## Building and Flashing

To build the current firmware and flash using a connected probe, install
[`cargo-embed`](https://crates.io/crates/cargo-embed) and then run:

```
cargo embed --release
```
