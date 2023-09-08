import serial as serial
import serial.tools.list_ports
import time
from infi.devicemanager import DeviceManager

def main():
    #ports = serial.tools.list_ports.comports()
    #print(ports)
    ser = serial.Serial(port = 'COM3', timeout = 0.1, xonxoff=False, rtscts=False, dsrdtr=False)
    ser.baudrate = 115200
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1

    time.sleep(1)
    
    print("start")
    ser.write(b'test')
    print("done")
    ser.close()
    
if __name__ == "__main__":
    main()