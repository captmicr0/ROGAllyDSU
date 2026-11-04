# Add standalone Windows EXE builds to captmicr0/ROGAllyDSU

Commit and push to `main`, or select **Actions > Build Windows standalone EXE > Run workflow**.
When it succeeds, download the `ROGAllyDSU-Windows-x64` artifact from the workflow run.
The artifact contains a ZIP with `ROGAllyDSU.exe` and `README.txt`.

The build uses Python only inside the GitHub Actions runner. The shipped EXE embeds Python and winsdk.

## Why a new entry point?

`ally_dsu.py` calls `mp.freeze_support()` *inside `main()`*, after `argparse`.
In a PyInstaller multiprocessing worker, that is too late: the frozen program
sees internal `--multiprocessing-fork` arguments before worker redirection.
The wrapper calls `freeze_support()` **before importing or invoking `main()`**.
This preserves the process-isolated WinRT sensor worker that fixed stationary stalls.

## Notes

- The workflow runs protocol tests, checks `--help`, and exercises the packaged EXE in `--demo` mode with the repository's `probe_dsu.py`.
- It cannot validate access to a physical ROG Ally gyroscope on a GitHub-hosted runner; test the release on your Ally.
- Do not keep the original Python server running alongside the EXE; both use UDP port 26760.
- For distribution, unsigned EXEs may trigger Windows SmartScreen warnings.
- Version tags beginning with `v` also publish the ZIP as a GitHub release. Other runs upload an Actions artifact.
