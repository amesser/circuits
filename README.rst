SDCard Data Logger
==================

This project describes a device designed to measure analog 
and digital signals and storing the values to a SDCard.

Features
--------

- Measurement interval configurable from 1ms to 60s in 1ms
  granularity.
- 4 analog channels and 4 digital inputs available. Free 
  configuration of the channels to use.
- Support for SDCard up to 2GB Memory.

Licenses
--------

Different Licenses are applied by this project. For Schematic 
and Board within subfolder ``board`` the following licensing 
conditions hold:

Copyright (c) 2013 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution-ShareAlike 3.0 Unported License. To view 
a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

The software of the microcontroller located in subfolder 
``firmware`` is licensed under the conditions of GPL V3.

Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Generate Firmware
-----------------

In order to generate the firmware, the following tools are
required:

- avr-gcc
- waf meta build system

Enter the firmware directory and execute::

  waf configure build

The firmware is located in the ``build`` subdirectory afterwards. As
an crystal is used for clocking the ATTiny2313 the following fuse settings
shall be applied::

  Low Fuse:      0x7D
  High Fuse:     0xDF
  Extended Fuse: 0xFF


Author
------

Andreas Messer <andi@bastelmap.de>


