@echo off
rem Double-click to open the robot car control panel in the browser.
rem Keep this window open while you use the panel; close it to stop.
cd /d "%~dp0"
python web_panel.py %*
if errorlevel 1 pause
