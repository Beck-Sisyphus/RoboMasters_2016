# To be run in conjunction with SerialPytArd_Test.ino
# Arduino file


import serial
import time



i = 0
a = -3.14159
b = 123.456
c = 3.14159

a = a * 100
b = b * 100
c = c * 100

a = int(a)
b = int(b)
c = int(c)

a1 = (a >> 8) & 255
a2 = (a & 255)
b1 = (b >> 8) & 255
b2 = b & 255
c1 = (c >> 8) & 255
c2 = c & 255

tx = [0] * 16
rx = [0] * 16


arduinoData = serial.Serial('com7', 115200)

# Send information to arduino
def arduinoTX():
    if i % 3 == 0:
        tx[4] = a1
        tx[5] = a2
        arduinoData.write(tx)
    elif i % 3 == 1:
        tx[4] = b1
        tx[5] = b2
        arduinoData.write(tx)

    else:
        tx[4] = c1
        tx[5] = c2
        arduinoData.write(tx)

# receive information from arduino
def arduinoRX():
    myData = arduinoData.read(16)

    # change string representation or rx data to int
    for j in range(len(myData)):
        rx[j] = ord(myData[j])
        
    packet = (( (rx[4] << 8)) | (rx[5] & 255))

    # to get correct negative representation of data in Python
    packet = twosComp(16, packet)
    
    if packet == -314:
        print "AAAAA"
    elif packet == 12345:
        print "BBBBB"

    elif packet == 314:
        print "CCCCC"
    else:
        print "Problem"


# Change negative numbers to
# correct readable representation in Python
def twosComp(bits, value):
    #if value is negative
    if (value >> 15) == 1:
        value = ~value + 1
        return -(( 1<<bits ) + value)
    else:
        return value
    #formatstring = '{:0%ib}' % bits
    #return formatstring.format(value)


# Continuously writes to Arduino
# and read the same info from Arduino
while (1 == 1):
    #write to arduino data
    arduinoTX()
    
    #receive arduino data
    if(arduinoData.inWaiting() > 0):
        arduinoRX()
    i = i + 1
    time.sleep(1)
        
