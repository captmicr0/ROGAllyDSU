# ROGAllyDSU Direct Release Assets Workflow

Copy `.github/workflows/build-windows.yml` into your repository, replacing the old file.

- Each successful push to `main` creates a `build-N` prerelease with `ROGAllyDSU.exe` and `README.txt` as separate downloadable files.
- Pushing a `v*` tag creates a normal release with the same separate assets.
- Pull requests build and test, but do not publish.
- Actions build artifacts remain available as a ZIP.

The workflow expects the repo to contain `frozen_entry.py`, `test_ally_dsu.py`, and `probe_dsu.py` (as in the previous working configuration).

Ensure Settings > Actions > General > Workflow permissions allows the GitHub Actions token to write contents.
