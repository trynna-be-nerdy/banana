# PROS Setup (Windows)

This repo is already a PROS project (`project.pros` exists).

## 1) Install PROS CLI
1. Install Python 3.11+.
2. Install PROS CLI:
   ```powershell
   py -m pip install pros-cli
   ```
3. Verify:
   ```powershell
   pros --version
   ```

## 2) Build / Upload
Run from this folder:
```powershell
pros make
pros upload
pros terminal
```

## 3) VS Code workflow
Open this folder in VS Code and run:
- `PROS: Build`
- `PROS: Upload`
- `PROS: Build + Upload`
- `PROS: Terminal`

These tasks are defined in `.vscode/tasks.json`.
