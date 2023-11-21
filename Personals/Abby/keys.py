import keyboard
import serial 

ser = serial.Serial(port="COM3", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, xonxoff=False, rtscts=False, dsrdtr=False) # xonxoff=False, rtscts=False, dsrdtr=False) #, 115200, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.close()
ser.open()
ser.flush()

prev_val = "stop"
cur_val = "stop" 

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
        cur_val = "up"
        if(cur_val != prev_val):
            serial_write(ser, "up")
            prev_val = "up"
    elif(keyboard.is_pressed("down") and (keyboard.is_pressed("up") == False and keyboard.is_pressed("left") == False and keyboard.is_pressed("right") == False)):
        cur_val = "down"
        if(cur_val != prev_val):
            serial_write(ser, "down")
            prev_val = "down"
    elif(keyboard.is_pressed("left") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("up") == False and keyboard.is_pressed("right") == False)):
        cur_val = "left"
        if(cur_val != prev_val):
            serial_write(ser, "left")
            prev_val = "left"
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("left") == False and keyboard.is_pressed("up") == False)):
        cur_val = "righ"
        if(cur_val != prev_val):
            serial_write(ser, "righ")
            prev_val = "righ"
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("up") and (keyboard.is_pressed("down") == False and keyboard.is_pressed("left") == False))):
        cur_val = "ur"
        if(cur_val != prev_val):
            serial_write(ser, "ur")
            prev_val = "ur"
    elif(keyboard.is_pressed("right") and (keyboard.is_pressed("down") and (keyboard.is_pressed("up") == False and keyboard.is_pressed("left") == False))):
        cur_val = "dr"
        if(cur_val != prev_val):
            serial_write(ser, "dr")
            prev_val = "dr"
    elif((keyboard.is_pressed("left") and (keyboard.is_pressed("down")) and (keyboard.is_pressed("up") == False and keyboard.is_pressed("right") == False))):
        cur_val = "dl"
        if(cur_val != prev_val):
            serial_write(ser, "dl")
            prev_val = "dl"
    elif((keyboard.is_pressed("left") and keyboard.is_pressed("up")) and (keyboard.is_pressed("right") == False and keyboard.is_pressed("down") == False)):
        cur_val = "ul"
        if(cur_val != prev_val):
            serial_write(ser, "ul")
            prev_val = "ul"
    else:
        cur_val = "stop"
        if(cur_val != prev_val):
            serial_write(ser, "stop")
            prev_val = "stop"

