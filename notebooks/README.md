# 📓 How to Add New Notebooks

## Quick Start (3 Steps)

Adding a new notebook to the GitHub Pages deployment is as simple as 1-2-3:

### Step 1: Create Your Notebook
```bash
uv run marimo edit notebooks/my_awesome_notebook.py
```

### Step 2: Commit and Push
```bash
git add notebooks/my_awesome_notebook.py
git commit -m "Add my awesome notebook"
git push origin main
```

### Step 3: Wait for Deployment
That's it! The GitHub Actions workflow will automatically:
- ✅ Detect your new notebook
- ✅ Export it to HTML-WASM format
- ✅ Deploy it to GitHub Pages
- ✅ Add it to the index page

Your notebook will be live at: `https://<username>.github.io/<repository-name>/my_awesome_notebook.html`

For this repository, it would be: `https://bsprenger.github.io/marimo-notebooks/my_awesome_notebook.html`

## Example: Currently Deployed Notebooks

As of now, these notebooks are automatically deployed:
- `example_visualization.py` → `https://<username>.github.io/<repository>/example_visualization.html`
- `robot_kinematics.py` → `https://<username>.github.io/<repository>/robot_kinematics.html`

(Replace `<username>` and `<repository>` with your GitHub username and repository name)

## Visual Flow

```
notebooks/
├── example_visualization.py    ──┐
├── robot_kinematics.py          ─┤
└── your_new_notebook.py ← ADD   ─┤
                                   │
                    Push to main   │
                         ↓         │
            [GitHub Actions]       │
                         ↓         │
         Export to HTML-WASM ←─────┘
                         ↓
         Create index.html
                         ↓
        Deploy to GitHub Pages
                         ↓
    https://<username>.github.io/<repository>/
```

## What Gets Deployed?

- ✅ All `.py` files in the `notebooks/` directory
- ❌ Files named `__init__.py` (automatically excluded)
- ❌ Files in subdirectories (not currently supported)

## Workflow Features

### Automatic Index Page
The workflow creates a beautiful landing page (`index.html`) that:
- Lists all available notebooks
- Provides direct links to each notebook
- Has a clean, modern design
- Updates automatically when you add notebooks

### Export Settings
Each notebook is exported with:
- **Format**: HTML-WASM (runs entirely in the browser)
- **Mode**: `run` (interactive read-only mode, not edit mode)
- **Self-contained**: No server needed, works offline after loading

## Advanced: Manual Deployment

Trigger the workflow manually:
1. Go to your repository's Actions tab: `https://github.com/<username>/<repository>/actions`
2. Select "Deploy Marimo Notebooks to GitHub Pages"
3. Click "Run workflow" → Select `main` branch → Click "Run workflow"

## Troubleshooting

### My notebook isn't showing up
- ✅ Check it's in the `notebooks/` directory (not a subdirectory)
- ✅ Verify it has a `.py` extension
- ✅ Make sure it's not named `__init__.py`
- ✅ Check the Actions tab for workflow status

### Workflow is failing
- View the workflow logs in the Actions tab
- Common issues:
  - Missing dependencies in `pyproject.toml`
  - Syntax errors in the notebook
  - Runtime errors when exporting

### Testing locally before deployment
```bash
# Test that your notebook runs without errors
uv run marimo run notebooks/your_notebook.py

# Test the export command (requires marimo)
uv run marimo export html-wasm notebooks/your_notebook.py -o test.html --mode run
```

## Need More Help?

See the detailed documentation in [`.github/DEPLOYMENT.md`](DEPLOYMENT.md)
