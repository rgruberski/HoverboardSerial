# HoverBoard Serial Port Client

### !!! Work in progress !!!

Simple steering of Hoverboard motors by serial port communication. Implementation with C++ with Boost libraries.

Hoverboard driver firmware: https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC
Inspirations and pieces of code: https://github.com/AlfreddGco/sonnybot-

Before running, you have to set the correct name of the serial port interface.

```bash
$ mkdir build
$ cd build
$ cmake 
$ make
$ ./HoverboardSerial
```

The code works well with USB to TTL adapter with a 3.3V voltage level.

Please ask if you have any questions.

https://www.linkedin.com/in/rgruberski/