@echo off
batchisp -device AT32UC3A0512 -hardware usb -operation onfail abort memory FLASH erase F loadbuffer "proto2\Debug\Proto2.hex" program verify start reset 0
