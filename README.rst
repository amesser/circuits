RS232 Infrared Receiver
=======================

This circuit implements a simple Infrared Receiver. It can be used to 
provide input data to the lirc daemon. The circuit is powered from 
RS232 interface itself. The length of Infrared pulses and spaces
is measured with a resolution of about 1us and transmitted through RS232
into the computer. 
I build the device because it turned out that the standard lirc homebrew
IR receiver for the serial port does not work properly in all situations
for me. If a video was played, the computer was too busy the get the 
correct timings and I had to press the remotes buttons several times 
until recognized.

Licenses
--------

For the Schematic and Board within subfolder ``board`` the following 
licensing conditions hold:

Copyright (c) 2015 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution-ShareAlike 3.0 Unported License. To view 
a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Author
------

Andreas Messer <andi@bastelmap.de>

