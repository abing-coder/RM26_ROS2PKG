@echo off
:START
start /b multichannel_isotp_txrx_cpp.runtx.bat
start /b /wait multichannel_isotp_txrx_cpp 0 rx 100
ping 127.0.0.1 -n 3 >nul