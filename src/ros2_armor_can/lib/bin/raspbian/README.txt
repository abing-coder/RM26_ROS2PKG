Launch the bmapi_test example directly: 
sudo ./run.sh

Or you might want to run your own app following the steps below:
1. Copy the library files below to /usr/local/lib/:
libusb-1.0.so
libbmapi.so
2. Grant access to the libraries:
chmod 777 /usr/local/bin/libusb-1.0.so
chmod 777 /usr/local/bin/libbmapi.so
3. Set LD_LIBRARY_PATH environment variable, and run your application with sudo:
sudo env LD_LIBRARY_PATH=/usr/local/lib/ ./your_app_name
!!!IMPORTANT!!! If you do not run your app using sudo(or under su session), BM_OpenEx() would return -3 because it does not have access to USB devices.