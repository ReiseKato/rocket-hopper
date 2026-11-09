# Rocket Hopper

`rocket-hopper` is a research and development project for a small reusable thrust-vector-controlled hopper. The long-term goal is to combine simulation, hardware tests, system identification, and real flight data into a progressively improving digital twin.

The repository is intended as a monorepo. The first implemented software component is the fast, mathematically transparent simulator in [`fast_sim/`](fast_sim/).

## Current scope

The current Fast Simulator contains only the first 2D rigid-body dynamics model:

- horizontal and vertical translation,
- pitch motion in one plane,
- thrust and gimbal angle as physical inputs,
- explicit rigid-body parameters,
- deterministic Newton-Euler dynamics,
- physics-focused unit tests.

Actuator dynamics, numerical integration, controllers, logging, visualization, Monte-Carlo studies, and MuJoCo integration are deliberately kept outside this first implementation and will be added step by step.

## Documentation

The complete project documentation lives in [`docs/`](docs/) and is built with MkDocs Material.

After installing the documentation dependencies, run from the repository root:

```bash
mkdocs serve
```

Then open the local address printed by MkDocs (normally `http://127.0.0.1:8000`).

See [`docs/development/setup.md`](docs/development/setup.md) for setup instructions.
