import serial

cmd_hex = [0x1b, 0xa1, 0x00, 0x40, 0xbf, 0x05]
Get_Navigation = bytes(cmd_hex) 

ser = serial.Serial("/dev/ttl_magnetic", baudrate=115200, timeout=0.01)
        
# PID constants
P = 0;
I = 0;
D = 0;
KP = 0.1  # Proportional gain
KI = 0.0002  # Integral gain
KD = 60  # Derivative gain

error = 0
lastError = 0
sensors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

left_speed = 0.00
right_speed = 0.00
base_speed = 0.6
max_speed = 0.6


while True:
        ser.write(Get_Navigation)
        ser.flush()
        reading = ser.readline()
        val1 = reading[3]
        val2 = reading[4]
        con1=bin(val1)[2:].zfill(8)
        con2=bin(val2)[2:].zfill(8)
        
        for i in range(8):
            sensors[i] = int(con1[i])
        for i in range(8):
            sensors[i+8] = int(con2[i])
  
        print(sensors)
        # positions = 0
        # positions = sensors[0] * 0.6 + sensors[1] * 0.5625 + sensors[2] * 0.525 + sensors[3] * 0.4875 + sensors[4] * 0.45 + sensors[5] * 0.4125 +sensors[6] * 0.375 + sensors[7] * 0.3375 + sensors[8] * 0.3 + sensors[9] * 0.2625 + sensors[10]* 0.225 + sensors[11]* 0.1875 + sensors[12] * 0.15 + sensors[13] * 0.1125 + sensors[14]* 0.075 + sensors[15] * 0.0375
        if sensors[0] == 0 and sensors[1] == 0 and sensors[2] == 0 and sensors[3] == 0 and sensors[4] == 0 and sensors[5] == 0 and sensors[6] == 0 and sensors[7] == 0 and sensors[8] == 0 and sensors[9] == 0 and sensors[10] == 0 and sensors[11] == 0 and sensors[12] == 0 and sensors[13] == 0 and sensors[14] == 0 and sensors[15] == 0 :
            positions = 0.3
        else:        
            positions = (sensors[0] * 0.6 + sensors[1] * 0.5625 + sensors[2] * 0.525 + sensors[3] * 0.4875 + sensors[4] * 0.45 + sensors[5] * 0.4125 +sensors[6] * 0.375 + sensors[7] * 0.3375 + sensors[8] * 0.3 + sensors[9] * 0.2625 + sensors[10]* 0.225 + sensors[11]* 0.1875 + sensors[12] * 0.15 + sensors[13] * 0.1125 + sensors[14]* 0.075 + sensors[15] * 0.0375) / (sensors[15] + sensors[14]+ sensors[13]+ sensors[12]+ sensors[11]+ sensors[10]+ sensors[9]+ sensors[8]+ sensors[7]+ sensors[6]+ sensors[5]+ sensors[4]+ sensors[3]+ sensors[2]+ sensors[1]+ sensors[0])
        error = positions - 0.3
        P = error
        I = I + error
        D = error - lastError
        motor_speed = KP* P + KI * I +KD * D
        lastError = error
        
        right_speed = base_speed + motor_speed
        left_speed = base_speed - motor_speed
        
        if right_speed > max_speed:
            right_speed = max_speed
        if right_speed < 0:
            right_speed = 0
        if left_speed < 0: 
            left_speed = 0
        print(str(left_speed)+"||"+str(right_speed))
