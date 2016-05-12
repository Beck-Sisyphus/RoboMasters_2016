# RoboMasters_2016
Arduino on TX1/Ubuntu

### Link with tutorisl
http://blog.opensensors.io/blog/2014/09/13/getting-started-with-arduino-on-linux/
In TX1, Arduino files are under: /usr/share/arduino


### Differences between TX1 and other environments
TX1: have to cast tx and rx char arrays in write() and readBytes() functions in Arduino code
write((uint8_t*) tx, 16);
readBytes((char*)rx, 16);

This casting also works in Windows, and non-TX1 linux environments.