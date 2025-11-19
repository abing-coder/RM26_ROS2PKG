import can
import udsoncan
from udsoncan.connections import BmcanConnection
from udsoncan.client import Client
from udsoncan.exceptions import *
from udsoncan.services import *

def myalgo(level, seed, params):
    """
    Builds the security key to unlock a security level. Returns the seed xor'ed with pre-shared key in this demo.
    You might want to implement your own algorithm according to OEM specs.
    """
    output_key = bytearray(seed)
    xorkey = bytearray(params['xorkey'])
 
    for i in range(len(seed)):
       output_key[i] = seed[i] ^ xorkey[i%len(xorkey)]
    return bytes(output_key)

config = dict(udsoncan.configs.default_client_config)
config['p2_timeout'] = 10
config['security_algo'] = myalgo
config['security_algo_params'] = dict(xorkey=b'\x12\x34\x56\x78')
config['data_identifiers'] = {
    0xF190 : udsoncan.AsciiCodec(15)       # Codec that read ASCII string. We must tell the length of the string
}

if __name__ == "__main__":
    udsoncan.setup_logging()
    bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    conn = BmcanConnection('can0', bus=bus, tester_msg_id=0x7DF, ecu_msg_id=0x64E, fd=False, dlc=8, padding=0xCC)
    #conn = BmcanConnection('can0', bus=bus, tester_msg_id=0x14DA94F1, ecu_msg_id=0x14DAF194, fd=False, dlc=8, padding=0xCC)
    with Client(conn,  request_timeout=10.0, config=config) as client:
        try:
            print('Change session.')
            client.change_session(DiagnosticSessionControl.Session.extendedDiagnosticSession)  # integer with value of 3
            print('Change security level.')
            client.unlock_security_access(1)   # Fictive security level. Integer coming from fictive lib, let's say its value is 5
            vin = client.read_data_by_identifier_first(0xF190)
            print('Old VIN:' + vin)  # 'ABCDE0123456789' (15 chars)
            client.write_data_by_identifier(udsoncan.DataIdentifier.VIN, 'ABC123456789')       # Standard ID for VIN is 0xF190. Codec is set in the client configuration
            print('Vehicle Identification Number successfully changed.')
            client.ecu_reset(ECUReset.ResetType.hardReset)  # HardReset = 0x01
        except NegativeResponseException as e:
            print('Server refused our request for service %s with code "%s" (0x%02x)' % (e.response.service.get_name(), e.response.code_name, e.response.code))
        except InvalidResponseException as e:
            print('Server sent an invalid payload : %s' % e.response.original_payload)
        except UnexpectedResponseException as e:
            print('Server sent an unexpected payload : %s' % e.response.original_payload)