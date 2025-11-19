#!/usr/bin/env python

"""
Shows how the receive messages via polling.
"""

import can
from can.bus import BusState


def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""

   
    print('Open BUSMUST CAN channel using 500kbps baudrate ...')
    with can.interface.Bus(
       bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True
    ) as bus:
        # bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=250000)
        # bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)
		#bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
        # set to read-only, only supported on some interfaces
        bus.state = BusState.PASSIVE
        
        print('Waiting for RX CAN messages ...')
        try:
            while True:
                msg = bus.recv(1)
                if msg is not None:
                    print(msg)
        except KeyboardInterrupt:
            pass  # exit normally


if __name__ == "__main__":
    receive_all()
