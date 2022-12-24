import serial
 
RS485 = serial.Serial(
    port='COM5',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
IDs_list = ["0"]
while True:
     IDs = str(RS485.readline().decode())
     print(IDs)          



