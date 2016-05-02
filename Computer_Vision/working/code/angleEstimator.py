
import math

# For this code to work, supply focal length and distance
# 
# Output angle in radian
# horangle outputs negative radians to rotate left
#          outputs positive radians to rotate right

# verangle outputs negative radians to move up
#          outputs positive radians to move down

FOCAL_LENGTH_IPHONE = 28.0



def verangle(origin, actual, distance):
	yd = actual - origin
	yd = yd / FOCAL_LENGTH_IPHONE
	return math.atan(yd/distance)


def horangle(origin, actual, distance):
	xd = actual - origin #horizontal distance
	xd = xd / FOCAL_LENGTH_IPHONE
	return math.asin(xd/distance)
	

if __name__=='__main__':
	rad = verangle(1160, 1771, 100.0)
	print math.degrees(rad)
