Soil Moisture Sensor
====================

Introduction
------------
At some time in 2013 I came up with the idea of an automatic solar powered 
watering system for my gardens. In the very first attempt a simple time based 
solution was used along with standard solor charge controller, lead acid battery 
and solar panel. By the time it turned out, that just enabling the pump 
one to two hours every morning becomes problematic when the wheater is wet. The 
battery is completely discharged as there is not enough sun available. But in 
that case there is no need to enable the pump at all, as it is raining. Thus
the watering should be coupled to the moist in the soil.

I have analyzed several soil moisture attempts and end up developing my own 
solution. The system consists of a tiny sensor board which is tu be plugged 
into the earth and a controller board which will read out the sensors and
control the pumps. Furthermore a calibration tool was created used to calibrate 
the sensor. Aside from soil moisture the sensor can also 'measure' temperature and
light intensity.

General Concepts
----------------

Right after powerup the sensor will perform the measurements. Afterwards the results 
are transmitted to the controller using a one directional uart protocol. The uart signal is
modulated as current onto the two-wire connection of the sensor. The sensor will repeat the 
transmission infinitely until its powered down.

In order to protect the sensor circuitry from environment the sensor board is coated with 
varnish and epoxy. This treatment and manufacturing tolerances of the components require a 
calibration of the sensor. The calibration data must be stored in each sensor individually. 
For that purpose the additional calibration tool has been created. The tool will record
the sensors raw measurement data along with reference data in order to compute the 
calibration data. The calibration data will be transferred into the sensor over the 2-wire
connection using a voltage modulation of the supply voltage.


Licenses
--------

The software located within the subfolder ``firmware`` the conditions of the 
GNU General Public License (GPL) apply. See file COPYING for more details.

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