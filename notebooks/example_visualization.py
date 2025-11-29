"""Example marimo notebook demonstrating basic visualization capabilities."""

import marimo

__generated_with = "0.9.0"
app = marimo.App(width="medium")


@app.cell
def __():
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Wedge

    return Circle, Wedge, mo, np, plt


@app.cell
def __(mo):
    mo.md(
        r"""
        # Example Marimo Notebook

        This notebook demonstrates the basic setup with:
        - marimo for interactive notebooks
        - numpy for numerical computing
        - matplotlib for visualization
        """
    )
    return ()


@app.cell
def __(Circle, Wedge, np, plt):
    # Create a simple visualization with Circle and Wedge patches
    fig, ax = plt.subplots(figsize=(8, 8))

    # Add a circle
    circle = Circle((0.5, 0.5), 0.3, fill=False, edgecolor="blue", linewidth=2)
    ax.add_patch(circle)

    # Add a wedge (pie slice)
    wedge = Wedge((0.5, 0.5), 0.2, 0, 90, facecolor="lightblue", edgecolor="blue")
    ax.add_patch(wedge)

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal")
    ax.set_title("Example: Circle and Wedge Patches")

    fig
    return ax, circle, fig, wedge


if __name__ == "__main__":
    app.run()
