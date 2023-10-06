import serial.tools.list_ports
import serial 
import serial.tools.list_ports
import time

ser = serial.Serial(port="COM6", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.close()
ser.open()
ser.flush()

while(True):
    ser.flush()
    raw_data = input("enter data: ")
    data_to_send = str((int(raw_data, base=2) << 4))
    while(len((data_to_send)) < 4):
        data_to_send = "0" + data_to_send
    data_to_send = data_to_send + "\n\r"
    data_to_send = data_to_send.encode("ascii")
    print(data_to_send)
    ser.write(data_to_send)
    time.sleep(.5)
    print("read: " + str(ser.readline()))