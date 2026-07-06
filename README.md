# FIFO-Based Hardware Implementation of an LZ77 Data Compressor
**Authors:** Divisht Dahiya (IIT Gandhinagar), Aryan Mor(NIT Kurukshetra)


## Project Overview

This repository implements the [LZ77](https://en.wikipedia.org/wiki/LZ77_and_LZ78) lossless data-compression algorithm in hardware, with the **FIFO buffer as the central building block**. LZ77 works by sliding a window over the input and replacing repeated byte sequences with `(offset, length, next_char)` tokens that point back into recently seen data. That sliding window is exactly a first-in-first-out queue, so a well-designed FIFO is the natural engine for the algorithm.

The project is built in two layers that mirror each other:

- **A software reference model in C**, which pins down the exact token grammar and decoder behaviour (including self-overlapping `length > offset` / run-length matches) and self-verifies through compress → decompress round-trips.
- **A synthesizable Verilog implementation**, which realises the same codec on hardware alongside a small library of reusable FIFOs — single-clock, dual-clock (clock-domain-crossing), and streaming (valid/ready) — that supply the sliding window and let compressor and decompressor stages be chained together.

Every FIFO and every LZ77 codec ships with a self-checking testbench, so the whole repository can be built and verified from a single `make`.

## 📁 Repository Structure

```text
├── sw/                              # C reference / software models
│   ├── lz77_compressor.c            # Classic array-based LZ77 (token dump demo)
│   ├── lz77_fifo_implementation.c   # LZ77 whose sliding window IS a circular FIFO
│   └── lz77_robust.c                # Self-verifying codec: compress + decompress round-trip
├── rtl/                             # Synthesizable Verilog
│   ├── fifo/
│   │   ├── sync_fifo.v              # Single-clock FIFO (power-of-two depth, registered read)
│   │   ├── async_fifo.v             # Dual-clock CDC FIFO (Gray-code, Cummings-style)
│   │   ├── stream_fifo.v            # FWFT valid/ready (AXI-Stream-like) FIFO with `last`
│   │   └── revised/
│   │       ├── revised_sync_fifo.v  # Sync FIFO for arbitrary (non-power-of-two) depth
│   │       └── revised_async_fifo.v # Async FIFO for arbitrary depth
│   └── lz77/
│       ├── lz77_classic.v           # Block / memory-mapped LZ77 codec (compress + decompress)
│       └── lz77_stream.v            # Streaming LZ77 codec (handshake, compress + decompress)
├── tb/                              # Self-checking testbenches
│   ├── tb_sync_fifo.v
│   ├── tb_async_fifo.v
│   ├── tb_revised_async_fifo.v
│   ├── tb_lz77_classic.v            # Drives lz77_classic.v
│   └── tb_lz77_stream.v             # Drives lz77_stream.v + stream_fifo.v
├── docs/                            # Notes, reports, diagrams
├── Makefile                         # Build C models + run every testbench
├── .gitignore                       # Ignore build artifacts / simulator output
└── README.md                        # Project documentation
```

## How the pieces fit together

The FIFO and the compressor are not separate concerns — the FIFO *is* the sliding window:

- **`sw/lz77_fifo_implementation.c`** makes this explicit: the search + look-ahead window is a circular ring buffer, and the algorithm runs entirely through `enqueue` / `dequeue` / `peek` on that FIFO.
- **`rtl/fifo/async_fifo.v`** bridges clock domains, so a compressor clocked at one rate can hand tokens to logic running at another (e.g. an I/O or link clock) without metastability, using Gray-coded pointers and two-flop synchronizers.
- **`rtl/fifo/stream_fifo.v`** provides a first-word-fall-through valid/ready interface carrying an end-of-stream `last` bit. It is what lets `lz77_stream_compress` → FIFO → `lz77_stream_decompress` be chained with proper backpressure, which is exactly how `tb_lz77_stream.v` exercises the streaming path.

The two RTL codecs offer two integration styles:

- **`lz77_classic.v`** is *block / memory-mapped*: load an input buffer, pulse `start`, wait for `done`, read the result. The compressed stream is a 4-byte little-endian original length followed by 3-byte `(offset, length, next_char)` tokens. It matches `sw/lz77_robust.c` byte-for-byte.
- **`lz77_stream.v`** is *dataflow*: bytes arrive on a handshake, tokens leave on a handshake, and only a sliding window is held — no whole frame is buffered.

Both handle the tricky overlap case (`length > offset`, i.e. RLE-style matches) because the decoder copies one byte per cycle and can read bytes it just produced.

## Building and running

Requirements: `gcc` for the C models and [Icarus Verilog](https://steveicarus.github.io/iverilog/) (`iverilog` + `vvp`) for the RTL simulations.

```bash
make            # build the C models and run all five testbenches
make c          # build only the C reference programs into build/
make sim        # run only the RTL testbenches
make sim_lz77_stream   # run a single testbench
make clean      # remove build artifacts
```

Each C program is a standalone demo with its own `main()`; after `make c` you can run them directly, e.g. `./build/lz77_robust`.

All testbenches are self-checking and print a clear `PASS` / `ROUND-TRIP OK` verdict. A full `make` run compiles the three C models and reports passing round-trips for the classic and streaming LZ77 codecs plus passing checks for the synchronous, asynchronous, and non-power-of-two FIFOs.

## A note on the "revised" FIFOs

`sync_fifo.v` and `async_fifo.v` assume a **power-of-two depth** and use the classic MSB-wrap-bit pointer trick. The versions under `rtl/fifo/revised/` relax that to **arbitrary depth** by tracking an explicit count / modulo-`DEPTH` address instead.

Because the revised modules keep the same module names (`sync_fifo`, `async_fifo`, `fifomem`) as the originals, do **not** compile an original and its revised counterpart into the same simulation — you will get duplicate-module errors. The provided `Makefile` compiles each testbench against exactly the design files it needs, so this only matters if you write your own build scripts.

## License

<!-- Add a license (e.g. MIT) if you intend others to reuse this code. -->
