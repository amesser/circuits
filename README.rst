USB Display Adapter
===================

This porjects aim is to create a generic USB adapter to connect HD44780
compatible displays to a pc. The adapter will use the USB HID Standard
Protocol (Page 0x14 for Alphanumeric Displays) and should therefore run 
out of the box with many LCD controlling applications.

Status
------

A initial version of this project is finished and running with a 16x2 
OLED display. Writing characters and Cursor Position is possible using
the USB HID protocol. Unfortunately i didn't found an existing usb hid 
alphanumeric display implementation in linux. I thus created a simple
LCDproc driver using hidraw interface of linux kernel. I thus can not 
verify if my USB HID descriptor and the implementation is valid 
USB HID alphanumeric display application.

Hardware
--------

The first version uses a USBN9604 chip for handling
the USB protocol and a ATTINY2313 for controlling the display
and doing the higher level usb protocol stuff. Currently about
1.9K Flash of the mcu are in use, but there is some potential
for optimization. The schematic/board has been prepared
to add additional devices via spi (buttons, leds... etc) and
an infrared receiver. In that case a larger mcu will be necessary.
A drop in replacement would by ATTiny 4313 with 4KB Flash.

Licenses
--------

For the Schematic and Board within subfolder ``board`` the following 
licensing conditions hold:

Copyright (c) 2013 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution-ShareAlike 3.0 Unported License. To view 
a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.


Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Author
------

Andreas Messer <andi@bastelmap.de>

