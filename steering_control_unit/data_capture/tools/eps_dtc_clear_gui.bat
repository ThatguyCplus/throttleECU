@echo off
rem EPS DTC clear window. Uses the Scope1008 venv (PySide6 + pyserial) when it sits next to throttleECU.
set "PY=%~dp0..\..\..\..\Scope1008claude\scope1008\.venv\Scripts\pythonw.exe"
if not exist "%PY%" set "PY=pythonw"
start "" "%PY%" "%~dp0eps_dtc_clear_gui.py" %*
