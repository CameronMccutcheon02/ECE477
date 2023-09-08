import serial 
import serial.tools.list_ports
import time

def main():
    ser = serial.Serial("COM6") #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
    ser.close()
    ser.open()
    while(True):
        ser.write(0x23)

if __name__ == "__main__":
    main()

'''
# DISABLED CODE TO IDENTIFY USB DEVICES
# CAN BE USED TO AUTO-DETECT THE WRITE PORT IN FUTURE 
    dm = DeviceManager()
    devices = dm.all_devices
    for d in devices:
        try:
            print(f'{d.friendly_name} : address: {d.address}, bus: {d.bus_number}, location: {d.location}' + "\n")
        except:
            pass
            
    #ports = serial.tools.list_ports.comports()
    #print(ports)
'''