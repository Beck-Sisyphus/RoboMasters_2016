import serial



arduinoData = serial.Serial('com7', 115200)


##def arduinoTX(index):
##    if index % 3 == 0:
##        tx[4] = a1
##        tx[5] = a2
##        arduinoData.write(tx)
##    elif index % 3 == 1:
##        tx[4] = b1
##        tx[5] = b2
##        arduinoData.write(tx)
##
##    else:
##        tx[4] = c1
##        tx[5] = c2
##        arduinoData.write(tx)


def arduinoRX():
    myData = arduinoData.read(16)
    
    if ord(myData[0]) == 97:
        print myData[0]
    


while (1 == 1):
    #receive arduino data
    if(arduinoData.inWaiting() > 0):
        arduinoRX()


        
