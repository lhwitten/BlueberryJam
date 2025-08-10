@echo off
:: Activate the conda environment
call conda activate blueberryjammin

:: Navigate to the directory containing main.py
cd /d "C:\Users\jdavis\Desktop\blueberry\BlueberryJam\src\Yolo"

:: Run the main.py script
python main.py

:: Pause the script to see any output before closing
pause