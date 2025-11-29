# marimo-notebooks

A collection of interactive [marimo](https://marimo.io/) notebooks for Python visualizations.

## Prerequisites

- Python 3.10 or higher
- [uv](https://docs.astral.sh/uv/) package manager

## Installation

Install uv if you haven't already:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Install dependencies:

```bash
uv sync
```

## Running Notebooks

To run a marimo notebook:

```bash
uv run marimo edit notebooks/example_visualization.py
```

Or to run in read-only mode:

```bash
uv run marimo run notebooks/example_visualization.py
```

## Project Structure

```
marimo-notebooks/
├── pyproject.toml          # Project configuration with uv toolchain
├── src/
│   └── marimo_notebooks/   # Shared Python package (if needed)
│       └── __init__.py
├── notebooks/              # Independent marimo notebooks
│   └── example_visualization.py
└── README.md
```

## Adding New Notebooks

Create new marimo notebooks in the `notebooks/` directory. Each notebook is independent and can be run separately.

## Dependencies

- **marimo**: Interactive Python notebooks
- **numpy**: Numerical computing
- **matplotlib**: Plotting and visualization

## License

MIT License