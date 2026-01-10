import can
import time
import os
import sys
import intelhex # pip install intelhex
import udsoncan
import ctypes
from udsoncan.connections import BmcanConnection
from udsoncan.client import Client
from udsoncan.exceptions import *
from udsoncan.services import *

def myalgo(level, seed, params):
    """
    Builds the security key to unlock a security level. Returns the seed xor'ed with pre-shared key in this demo.
    You might want to implement your own algorithm according to OEM specs.
    """
    iSeedArray = ctypes.c_char_p(bytes(seed))
    iSeedArraySize = ctypes.c_uint(len(seed))
    iSecurityLevel = ctypes.c_uint(level)
    iVariant = ctypes.c_char_p(0)
    ioKeyArray = ctypes.create_string_buffer(len(seed))
    iKeyArraySize = ctypes.c_uint(len(seed))
    oSize = ctypes.c_int(0)
    dll = ctypes.CDLL('./SeednKey.dll')
    result = dll.GenerateKeyEx(iSeedArray, iSeedArraySize, iSecurityLevel, iVariant, ioKeyArray, iKeyArraySize, ctypes.byref(oSize))
    if result != 0:
        raise Exception("GenerateKeyEx failed with error code %d." % result)
    
    output_key = ctypes.string_at(ioKeyArray, ctypes.sizeof(ioKeyArray))
    return bytes(output_key)

config = dict(udsoncan.configs.default_client_config)
config['security_algo'] = myalgo
config['security_algo_params'] = dict(xorkey=b'\x12\x34\x56\x78')
config['data_identifiers'] = {
    0xF190 : udsoncan.AsciiCodec(15)       # Codec that read ASCII string. We must tell the length of the string
}

if __name__ == "__main__":
    os.chdir(os.path.abspath(sys.path[0]))
    hex = intelhex.IntelHex('./ECU.hex')

    udsoncan.setup_logging()
    bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    conn = BmcanConnection('can0', bus=bus, tester_msg_id=0x7DF, ecu_msg_id=0x798, fd=True, dlc=8, padding=0xCC)
    with Client(conn,  request_timeout=10.0, config=config) as client:
        try:
            print('Change session.')
            client.change_session(DiagnosticSessionControl.Session.extendedDiagnosticSession)  # integer with value of 2
            print('Change security level.')
            client.unlock_security_access(1)   # Fictive security level. Integer coming from fictive lib, let's say its value is 5
            client.change_session(DiagnosticSessionControl.Session.programmingSession)  # integer with value of 2
            time.sleep(1.0)
        except NegativeResponseException as e:
            print('Server refused our request for service %s with code "%s" (0x%02x)' % (e.response.service.get_name(), e.response.code_name, e.response.code))
        except InvalidResponseException as e:
            print('Server sent an invalid payload : %s' % e.response.original_payload)
        except UnexpectedResponseException as e:
            print('Server sent an unexpected payload : %s' % e.response.original_payload)

        t0 = time.time()
        totallength = 0
        for segment in hex.segments():
            startaddr = segment[0]
            length = segment[1] - segment[0]
            data = bytes([hex[a] for a in range(segment[0], segment[1])])
            print('Download segment: addr=0x%08x, len=%u.' % (startaddr, length))
            try:
                response = client.request_download(udsoncan.MemoryLocation(address=startaddr, memorysize=length))
                blocklength = response.service_data.max_length
                nblocks = (length + blocklength - 1) // blocklength
                for i in range(nblocks):
                    client.transfer_data((i+1)&0xFF, data[i*blocklength : min((i+1)*blocklength, len(data))])
                client.request_transfer_exit()
            except NegativeResponseException as e:
                print('Server refused our request for service %s with code "%s" (0x%02x)' % (e.response.service.get_name(), e.response.code_name, e.response.code))
            except InvalidResponseException as e:
                print('Server sent an invalid payload : %s' % e.response.original_payload)
            except UnexpectedResponseException as e:
                print('Server sent an unexpected payload : %s' % e.response.original_payload)
            totallength += length
        elapsed = time.time() - t0

        print('Done, downloaded %d bytes in %.3f seconds.' % (totallength, elapsed))