import serial.tools.list_ports
from infi.devicemanager import DeviceManager
from serial_comms import setup_serialobject, serial_read, serial_write

ser = setup_serialobject()
serial_write(ser, b'01010101')
print(serial_read(ser))