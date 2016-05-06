Traffic counting device
=======================

This software implements a traffic counting device based on an ATmega328 
microcontroller. The device records various parameters in a logfile on an SD 
card for later evaluation. 

Status
------
The firmware is al most finished . Unfortunanetly there is currently no hardware 
schematic and pcb layout as the hardware is in development (breadboard) stage
at the moment. 

Programming
-----------

- AVR Dude Fuse settings (8 Mhz external crystal oscillator):
  -U lfuse:w:0x5d:m -U hfuse:w:0xd9:m -U efuse:w:0xfc:m 

Licenses
--------

The firmware is licensed under the GNU General Public License. Please see file 
LICENSE.GPL within root folder of repository for details.

Author
------
Andreas Messer <andi@bastelmap.de>

