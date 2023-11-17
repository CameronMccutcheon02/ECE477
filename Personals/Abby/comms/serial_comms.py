#######################################################
# file: serial_comms.py
# description: used in communicating with stm32
# author: Abby 
#######################################################
import serial 
import serial.tools.list_ports
import time
from infi.devicemanager import DeviceManager

def setup_serialobject():
    dm = DeviceManager()
    dm.root.rescan()
    devices = dm.all_devices
    for device in devices:
        if "USB Serial Port" in device.description:
            str = device.__str__()
            port = str.split('(', 1)[1].split(')')[0]
            print(port)

    ser = serial.Serial(port="COM6", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
    ser.close()
    ser.open()
    ser.flush()

    return ser

def serial_write(ser, data):
    ser.flush()
    data_to_send = str(data)
    while(len((data_to_send)) < 4):
        data_to_send = data_to_send + "0"
    data_to_send = data_to_send.encode("ascii")
    print(data_to_send)
    ser.write(data_to_send)
    time.sleep(.1)

def serial_read(ser):
    response = ser.readline()
    return response
