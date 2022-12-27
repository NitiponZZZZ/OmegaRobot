import serial
from PIL import Image
# RS485 = serial.Serial(
#     port='/dev/ttl_rfid',
#     baudrate = 9600,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=0.1
# )
# id_list =["E0840000134216067C","E0840000134017267D","E08400001338494970","E08400001337661678","E08300000197886572","E08400001341097878","E0840000134083977A","F99900180420040377","E08400001342519373","E08400001342905170"] 
# station = 0
img = Image.open('/home/robot5g/dev_ws/src/omega/omega/num2.jpg')

while True:
    img.show()

    # FIDs = str(RS485.readline().decode())
    # IDs = FIDs.strip('$').split('#')
    # # print(len(id_list))
    # print(IDs[0])

    # for i in range(0,len(id_list)):
    #     print(station)
    #     if str(IDs[0]) == str(id_list[i]):
    #         station = i
    #         print(id_list[i])
