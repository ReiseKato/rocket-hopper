# Installation and Local Development

## Prerequisites

The following are required:

- Git,
- Python 3.11 or newer,
- `pip` and `venv` (usually included with a Python installation).

The initial version of the physics core does not require NumPy as a dependency.

## Recommended Virtual Environment

From the root directory of the `rocket-hopper` repository:

=== "Linux / macOS"

    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install -e "./fast_sim[dev,docs]"
    ```

=== "Windows PowerShell"

    ```powershell
    py -m venv .venv
    .\.venv\Scripts\Activate.ps1
    python -m pip install --upgrade pip
    python -m pip install -e ".\fast_sim[dev,docs]"
    ```

The extras install the following dependencies:

- `dev`: `pytest`,
- `docs`: `mkdocs-material`, including compatible MkDocs and Markdown dependencies.

## Running Tests

```bash
python -m pytest fast_sim/tests
```

## Previewing the Documentation Locally

From the repository root:

```bash
mkdocs serve
```

MkDocs starts a local development server and automatically reloads the documentation when changes are made.

## Installing the Package in Editable Mode

The `-e` option installs the Fast Simulator in *editable mode*. This allows changes made under `fast_sim/src/` to be reflected directly in the virtual environment without having to reinstall the package after each modification.