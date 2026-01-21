# ChaosBSD

A Driver Proving Ground

ChaosBSD is a fork of FreeBSD. It exists because upstream cannot, and should not, accept broken drivers, half-working hardware, vendor trash, or speculative hacks.

We can.

## What This Is

- An abuse lab for hardware
- A staging ground for driver development: ports, reverse engineering, clean-room implementations
- A place for incomplete implementations to mature before upstream
- **amd64 + arm64**: modern x86_64 and ARM64 machines

This is a lab, not an OS.

## What This Is Not

- A desktop/server/enterprise OS
- A competitor to FreeBSD
- Stable

If ChaosBSD boots, that's a bonus, not a promise.

## Firmware Blobs

Blobs are allowed but must be marked with:
- Source (where it came from)
- License (or lack thereof)

No unmarked binaries.

## Driver Lifecycle

1. **Ingress** - compile, attach
2. **Stabilization** - stop panics, fix DMA, fix locking
3. **Sanitization** - clean, document, justify
4. **Exit** - upstream or delete

Drivers that stagnate are removed.

## Time Paradox

This repo lives in a time paradox.

Periodically, the timeline resets and ChaosBSD becomes FreeBSD again. This keeps cherry-picking with upstream simple. After reset, we restore it to the last chaotic state.

History may be rewritten. Commits may vanish. That's the deal.

## Similar Projects

People asked if there is a similar effort for Linux. There is: [t2sde](https://github.com/rxrbln/t2sde) - a more advanced framework for Linux.
