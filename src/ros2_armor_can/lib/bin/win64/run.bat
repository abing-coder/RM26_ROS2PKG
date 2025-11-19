@echo off
:START
start /b runtx.bat
start /b /wait bmapi_test 0 rx 1000
ping 127.0.0.1 -n 3 >nul
goto START