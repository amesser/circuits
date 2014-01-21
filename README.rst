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

This project will create a simple radar motion detector usable for 
speed measurement and detection of humans and animals.

Status
------

Electric circuit and board layout under development.

Hardware
--------

The project uses a 3-pin IPM-165 Dopplerradar sensor. Its output signal
is amplified by a low noise amplifier by 60, 80 and 100db amplification.
These three signals will be fed into an AVR ATMega8 MCU which will
perform the signal processing. An Nokia 5110 Display is attached
to the MCU to visualize results.

Licenses
--------

For the Schematic and Board within subfolder ``board`` and the spice
circuit description within ``spice`` subfolder the following 
licensing conditions hold:

Copyright (c) 2014 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution-ShareAlike 3.0 Unported License. To view 
a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

The model files within the ``spice`` subfolder ship with their own 
licenses. See the files for more details.

Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Author
------

Andreas Messer <andi@bastelmap.de>

