@echo off
rem Connects Android Studio to the robot (Control Hub) over Wi-Fi.
rem First join the robot's Wi-Fi network, then double-click this file.
rem After "connected to 192.168.43.1:5555" the Control Hub shows up in Android Studio's device list.
cd /d "%LOCALAPPDATA%\Android\Sdk\platform-tools"
adb connect 192.168.43.1:5555
adb devices
pause
