from os import getcwd
import glob
import random
import pdb

TRAIN = 0.5

def resample():
	pdb.set_trace()
	wd = getcwd()
	wd = wd + "/"
	train = open("train.txt", 'w')
	val = open("val.txt", 'w')
	pics = glob.glob("positive/*.jpg")
	index = range(len(pics))
	random.shuffle(index)
	size = int(len(pics) * 0.5)
	for i in range(size):
		train.write(wd + pics[index[i]] + "\n")
	train.close()
	for i in range(size, len(pics)):
		val.write(wd + pics[index[i]] + "\n")
	val.close()

resample()
