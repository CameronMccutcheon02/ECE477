import serial 
import serial.tools.list_ports
import time

ser = serial.Serial(port="COM3", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
while(1):
    ser.flush()
    data_to_send = input("enter data: ")
    while(len((data_to_send)) < 4):
        data_to_send = data_to_send + "0" 
    data_to_send = data_to_send.encode("ascii")
    print(data_to_send)
    ser.write(data_to_send)
    time.sleep(.1)

'''
list = ["-1", "1"]
i = 0
while i < 10:
    for x in list:
        ser.flush()
        print(x)
        serial_write(ser, x)
        time.sleep(.025)
    i = i + 1

ser = serial.Serial(port="COM6", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.close()
ser.open()
ser.flush()

vals = [b'4080', b'4064', b'4032', b'3968', b'3840', b'3584', b'3072', b'2048', b'0000', b'4080', b'4064', b'4032', b'3968', b'3840', b'3584', b'3072', b'2048', b'0000']
i = 0
while i < 3:
    for x in vals:
        ser.flush()
        print(x)
        ser.write(x)
        time.sleep(.025)
    i = i + 1

ser.flush()
data_to_send = input("enter data: ")
#data_to_send = str((int(raw_data, base=2)))
while(len((data_to_send)) < 4):
    data_to_send = "0" + data_to_send
data_to_send = data_to_send.encode("ascii")
print(data_to_send)
ser.write(data_to_send)
time.sleep(.1)
'''