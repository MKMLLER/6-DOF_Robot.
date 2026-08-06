@echo off
cd /d "%~dp0"
echo Launching Robotic Arm Control System...
if exist "C:\Users\mikem\AppData\Local\Programs\Python\Python311\python.exe" (
    "C:\Users\mikem\AppData\Local\Programs\Python\Python311\python.exe" main.py
) else (
    py -3 main.py || python main.py
)
pause

