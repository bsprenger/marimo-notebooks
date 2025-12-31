# Adding Notebooks to GitHub Pages Deployment

## How It Works

The GitHub Actions workflow automatically deploys all marimo notebooks from the `notebooks/` directory to GitHub Pages.

## Adding a New Notebook

**It's simple!** Just add your new marimo notebook (`.py` file) to the `notebooks/` directory and push to the `main` branch. The workflow will automatically:

1. Detect the new notebook
2. Export it to HTML-WASM format
3. Deploy it to GitHub Pages
4. Add it to the index page

### Steps:

1. Create your marimo notebook in the `notebooks/` directory:
   ```bash
   uv run marimo edit notebooks/my_new_notebook.py
   ```

2. Commit and push your changes:
   ```bash
   git add notebooks/my_new_notebook.py
   git commit -m "Add new notebook: my_new_notebook"
   git push origin main
   ```

3. That's it! The GitHub Actions workflow will automatically deploy your notebook.

## Viewing Your Deployed Notebooks

After the workflow completes:
- Visit your GitHub Pages URL (shown in the Actions tab after deployment)
- The index page lists all available notebooks
- Click on any notebook to view it interactively in your browser

## Manual Deployment

You can manually trigger the deployment workflow:
1. Go to the "Actions" tab in your GitHub repository
2. Select "Deploy Marimo Notebooks to GitHub Pages"
3. Click "Run workflow"

## Requirements

- Notebooks must be in the `notebooks/` directory
- Notebooks must be valid Python files with marimo app structure
- Notebooks should not be named `__init__.py` (this is excluded automatically)

## Troubleshooting

### Workflow fails during export
- Check that your notebook runs without errors: `uv run marimo run notebooks/your_notebook.py`
- Ensure all dependencies are listed in `pyproject.toml`

### Notebook not appearing on the deployed site
- Verify the notebook is in the `notebooks/` directory (not in a subdirectory)
- Check that the file has a `.py` extension
- Look at the workflow logs in the Actions tab for any errors

## Technical Details

The workflow:
- Uses `marimo export html-wasm` to create self-contained HTML files with WebAssembly
- The `--mode run` flag executes notebooks in run mode (not edit mode)
- Each notebook becomes a standalone interactive webpage
- An index page is automatically generated listing all notebooks
