import keyboard
import serial 
import time

ser = serial.Serial(port="COM3", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.close()
ser.open()
ser.flush()

def serial_write(ser, data):
    ser.flush()
    data_to_send = str(data)
    while(len((data_to_send)) < 4):
        data_to_send = data_to_send + "0"
    if(len(data_to_send) > 4):
        data_to_send = "0000"
    data_to_send = data_to_send.encode("ascii")
    print(data_to_send)
    ser.write(data_to_send)

while True:
    if(keyboard.is_pressed("up") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("left") == False and keyboard.is_pressed("right") == False)):
        print("up")
        serial_write(ser, "up")
    elif(keyboard.is_pressed("down") and (keyboard.is_pressed("up") == False and keyboard.is_pressed("left") == False and keyboard.is_pressed("right") == False)):
        print("down")
        serial_write(ser, "down")
    elif(keyboard.is_pressed("left") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("up") == False and keyboard.is_pressed("right") == False)):
        print("left")
        serial_write(ser, "left")
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("left") == False and keyboard.is_pressed("up") == False)):
        print("righ")
        serial_write(ser, "righ")
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("up") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("left") == False))):
        print("ur")
        serial_write(ser, "ur")
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("down") and (keyboard.is_pressed("up") == False and keyboard.is_pressed("left") == False))):
        print("dr")
        serial_write(ser, "dr")
    elif((keyboard.is_pressed("left") and (keyboard.is_pressed("down")) and (keyboard.is_pressed("up") == False and keyboard.is_pressed("right") == False))):
        print("dl")
        serial_write(ser, "dl")
    elif((keyboard.is_pressed("left") and keyboard.is_pressed("up")) and (keyboard.is_pressed("right") == False and keyboard.is_pressed("down") == False)):
        print("ul")
        serial_write(ser, "ul")
    else:
        print("stop")
        serial_write(ser, "stop")
