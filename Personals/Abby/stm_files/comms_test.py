import serial.tools.list_ports
from infi.devicemanager import DeviceManager
from serial_comms import setup_serialobject
import serial 
import serial.tools.list_ports
import time

ser = setup_serialobject()
ser.close()
ser.open()
ser.flush()

while(True):
    raw_data = input("enter data: ")
    data_to_send = str((int(raw_data, base=2) << 4))
    while(len((data_to_send)) < 4):
        data_to_send = "0" + data_to_send
    data_to_send = data_to_send.encode()
    print(data_to_send)
    ser.write(data_to_send)
    time.sleep(.5)