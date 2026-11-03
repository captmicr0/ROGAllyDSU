@echo off
cd /d "%~dp0"
py -3 ally_dsu.py %*
echo.
echo Server exited. Press a key to close.
pause >nul
