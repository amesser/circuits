USB Display Adapter
===================

This porjects aim is to create a generic USB adapter to connect HD44780
compatible displays to a pc. The adapter will use the USB HID Standard
Protocol (Page 0x14 for Alphanumeric Displays) and should therefore run 
out of the box with many LCD controlling applications.

Hardware
--------

The currently planned hardware is a USBN9604 chip for handling
the USB protocol and a ATTINY2313 for controlling the display
and doing the higher level usb protocol stuff.

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

