#######################################################
# file: serial_comms.py
# description: used in communicating with stm32
# author: Abby 
#######################################################
import serial 
import serial.tools.list_ports
import time
from infi.devicemanager import DeviceManager

def setup_
dm = DeviceManager()
dm.root.rescan()
devices = dm.all_devices
for device in devices:
    if "USB Serial Port" in device.description:
        str = device.__str__()
        port = str.split('(', 1)[1].split(')')[0]
        print(port)

ser = serial.Serial("COM6", 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.close()
ser.open()
ser.flush()

while True:
    ser.write("\ntest".encode())
    data = ser.readline()
    print(data)