import serial
import sys
import os

# PACKET TYPES
INITIATION = b'\x01\x00'
DATA = b'\x02\x00'
TERMINATION = b'\x03\x00'


if len(sys.argv) > 1:
    interface = sys.argv[1]
else:
    interface = '/dev/ttyUSB0'

if len(sys.argv) > 2:
    directory = sys.argv[2]
else:
    directory = ''


fileObj = None

with serial.Serial(interface, 9600, timeout=3600) as ser:
    while 1:
            print("Receiving Packet .. ")
            stream = b''
            count = 0

            msg_type = ser.read(2)
            print(msg_type)

            msg_len = ser.read(2)
            print(msg_len)
            msg_len = int.from_bytes(msg_len, byteorder='little')
            print(msg_len)

            while count < 76:
                x = ser.read(1)
                count += 1
                stream += x

            stream = stream[:msg_len]

            if (not fileObj) and msg_type == INITIATION:
                fileObj = open(os.path.join(directory, stream.decode()), 'wb')
            elif fileObj and msg_type == DATA:
                fileObj.write(stream)
            elif fileObj and msg_type == TERMINATION:
                fileObj.close()
                fileObj = None
            else:
                print("ERROR OCCURED")
