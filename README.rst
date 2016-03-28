Radar Detector
==============

Today radar sensors are becoming more and more popular. They are cheap,
allow measuring of speed, distance and are even able to detect multiple
objects at the same time. Of course such applications require complicated
setups but simple speed measurement can be done with a simple 3-pin
doppler radar. In contrast to PIR sensors radar sensors are sensitive to 
moving reflectors, eg. metalic objects or objects made of water (animals,
humans). Thus it is not as easy to fool these sensors and they often can
detect objects on larger distances. But they also have disadvantages: Detection
of objects moving parallel to the sensor is not possible with simple
doppler radars. Also the signal strength depends on the angle between the object
and the radar normal.

This project aims at creating building blocks for radar based devices.

Status
------

- A 2-channel radar amplifier and signal shaper board is ready to use.
- A traffic counter application build on top of the 2-channel frontend board
  is work in process. An initial firmware is available but no fixed board layout.

Hardware
--------

2-Channel amplifier & frontend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The 2-channel amplifier is designed for a 5-pin IPS-265 Dopplerradar sensor 
from InnoSent. But it can also be used with a single channel IPM-165. The 
amplification of the design is 80 to 84 dB. The amplified signal is provided 
as output. Additionally the amplified signal is passed to a schmitt trigger
and available as digital output.

Traffic counting device
~~~~~~~~~~~~~~~~~~~~~~~

The purpose of the traffic counting device is to record the time, speed, 
direction and length of a car or a lorry on a street. The device generates a
log record entry for each car or lorry onto an SD card.

Licenses
--------

For the Schematic and Board within subfolder ``board`` and the spice
circuit description within ``spice`` subfolder the following 
licensing conditions hold:

Copyright (c) 2014-2016 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution-ShareAlike 3.0 Unported License. To view 
a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

The model files within the ``spice`` subfolder ship with their own 
licenses. See the files for more details.

The script files within the ``scripts`` subfolder are licensed under the
GNU General Public License. Please see file LICENSE.GPL for details.

Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Author
------
Andreas Messer <andi@bastelmap.de>

