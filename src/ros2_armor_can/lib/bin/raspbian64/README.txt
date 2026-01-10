BUSMUST devices depend on libusb, please install libusb libraries if not available on your Linux system.
i.g. On Raspbian (64-bit), use apt-get to install libusb: 
sudo apt-get install libusb-1.0-0-dev

Launch the python "receive_all" example directly: 
cd $(YOUR_BMAPI_SDK_PATH)/example/python-can-3.3.4
sudo ./example.run.sh

Or you might want to run your own app following the steps below:
1. Copy the library files below to /usr/local/lib/:
libbmapi64.so [or libbmapi64d.so for debugging purpose]
2. Grant access to the libraries:
chmod 777 /usr/local/bin/libbmapi64.so
3. Set LD_LIBRARY_PATH environment variable, and run your application with sudo:
sudo env LD_LIBRARY_PATH=/usr/local/lib/ ./your_app_name
!!!IMPORTANT!!! If you do not run your app using sudo(or under su session), BM_OpenEx() would return -3 because it does not have access to USB devices.
