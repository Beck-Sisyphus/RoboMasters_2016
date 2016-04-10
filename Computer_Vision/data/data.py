
import subprocess as sub



if __name__ == '__main__':
	video = 'resource/BlueCar.avi'
	framesPerSecond = '5'
	outputFile = 'resource/blue/blue_%04d.png'
	command = ['ffmpeg', '-i', video, '-r', framesPerSecond, outputFile]
	sub.call(command)