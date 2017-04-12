import serial
import sys

if len(sys.argv) > 1:
    interface = sys.argv[1]
else:
    interface = '/dev/ttyUSB0'


with serial.Serial(interface, 9600, timeout=3600) as ser:
    stream = b''
    while 1:
        x = ser.read()
        if x == b'\x00':
            message = stream.decode("utf-8")
            print("RECEIVED: ", message)
            stream = b''
        stream += x          # read one byte
