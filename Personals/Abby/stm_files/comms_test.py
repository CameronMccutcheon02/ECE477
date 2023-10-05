import serial.tools.list_ports
from infi.devicemanager import DeviceManager
from serial_comms import setup_serialobject, serial_read, serial_write

ser = setup_serialobject()
ser.close()
ser.open()
ser.flush()
serial_write(ser, "11111111")

while(True):
    print(serial_read(ser))