import glob
import pdb

WID = 1280.0
HEI = 720.0

def convert():
	labels = glob.glob("*.txt")
	for label in labels:
		#pdb.set_trace()
		filename = str.split(label, '.')[0]
		with open(label) as f:
			content = f.readlines()
			category = int(content[0])
			coors = str.split(content[1], ' ')
			x1, y1, x2, y2 = coors
			width = int(x2) - int(x1)
			height = int(y2) - int(y1)
			x = int(x1) + width / 2.0
			y = int(y1) + height / 2.0
			output = open(filename + ".txt", 'w')
			output.write(str(category) + " " + str(x/WID) + " " + str(y/HEI) + " " + str(width/WID) + " " + str(height/HEI))
			output.close()
convert()
