@echo off
rem Shaddad UGV dashboard: start the local server and open http://localhost:8765 in the default browser.
rem Close this window to stop it. Nothing here writes to the project.
cd /d "%~dp0"
start "" /min cmd /c "timeout /t 4 >nul & start http://localhost:8765"
python server.py --port 8765
