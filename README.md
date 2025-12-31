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

## Viewing Notebooks Online

All notebooks are automatically deployed to GitHub Pages and can be viewed interactively in your browser without any local setup!

**[🌐 View Notebooks Online](https://bsprenger.github.io/marimo-notebooks/)**

## Adding New Notebooks

Create new marimo notebooks in the `notebooks/` directory. Each notebook is independent and can be run separately.

**To deploy a new notebook to GitHub Pages:** Simply add your `.py` notebook file to the `notebooks/` directory and push to the `main` branch. The GitHub Actions workflow will automatically export and deploy it.

See [`.github/DEPLOYMENT.md`](.github/DEPLOYMENT.md) for detailed deployment instructions.

## Dependencies

- **marimo**: Interactive Python notebooks
- **numpy**: Numerical computing
- **matplotlib**: Plotting and visualization

## License

MIT License