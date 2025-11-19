export LD_LIBRARY_PATH=../python-can-4.0.0/can/interfaces/bmcan:$PATH
export PYTHONPATH=.:../python-can-4.0.0:$PYTHONPATH
python3 ./examples/receive_all.py
