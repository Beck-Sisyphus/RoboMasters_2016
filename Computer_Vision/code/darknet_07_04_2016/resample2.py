from os import getcwd
from glob import glob
import random
import pdb

TRAIN = 0.5

def resample():
	#pdb.set_trace()
	wd = getcwd()
	wd = wd + "/"
	train = open("train.txt", 'w')
	pics = glob("blue_rover/*.jpg")
	#pics.extend(glob("yieldsign/*.JPEG"))
	index = range(len(pics))
	random.shuffle(index)
	size = int(len(pics))
	for i in range(size):
                print pics[index[i]]
		train.write(wd + pics[index[i]] + "\n")
	train.close()
	

resample()
