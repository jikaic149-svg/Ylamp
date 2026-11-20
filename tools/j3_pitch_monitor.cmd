@echo off
python "%~dp0j3_pitch_monitor.py" %*
if errorlevel 1 pause
