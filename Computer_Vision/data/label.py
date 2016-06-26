import subprocess
import glob
import pdb
import os

WID = 2988.0
HEI = 5312.0

def convert():
	pics = glob.glob("*.jpg")
	labeldir = "labels/"
	if not os.path.exists(labeldir):
		subprocess.call(["mkdir", labeldir])
	for pic in pics:
		filename = str.split(pic, '.')[0]
		one, two = str.split(filename, '_')
		onex, oney = str.split(one, ',')
		twox, twoy = str.split(two, ',')
		width = int(twox) - int(onex)
		height = int(twoy) - int(oney)
		x = int(onex) + width / 2.0
		y = int(oney) + height / 2.0
		output = open(labeldir + filename + ".txt", 'w')
		output.write("1 " + str(int(x)/WID) + " " + str(int(y)/HEI) + " " + str(width/WID) + " " + str(height/HEI))
		output.close()

convert()
