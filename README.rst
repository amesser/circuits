Low power sun tracker
=====================

After having played around with a lot of different solar panel sun tracker 
variations I have tried this new circuit. The need for that arose when recognizing
that the last attempt did not survive the winter.

This time I decided to build a design which should be better protectable
from environment influences. Additionally the device should consume
as low as possible power over night when solar panel is offline.

Design
------

Initially I was planning to integrate sensor and control unit on one small 
PCB. Thus the PCB has still places to mount the Sensor Leds. In my final setup
I don't use these and placed the sensor leds in a dedicaded housing. In my case this
was simpler and easier to mount. Nevertheless the sensor Leds can still mounted 
onto the PCB. The design was made based on following requirements:

- Motor should be simple brushed DC Motor
- At night panel should turn back to sun-set position. Thus Motor will be enabled
  at night. Mechanical setup requires positions switches to disable the motor at the
  end positions. Position switchs a bridged with a diode each to allow movement in opposite
  direction
- As low as possible power consumption, especially at night when no sun is 
  available. The H-Bridge for Motor was designed in that in idle mode, 
  motor is turned on. In Idle mode no active currents despite Mosfet Gate voltage limiting
  (Only required when V Supply > max Vgs) are consumed by H-Bridge 
- Use sourface mount only devices. This should led to better protection against 
  environmental influences. Initial idea was that sensor is made by backward mounted 
  Leds. (Looking through a hole in PCB each) and backside of PCB is entrely 
  covered by glas plate. In middle of glas plate a small shadow making metal plate
  was intended to cover inner sensors with sunlight shadow. It turned out
  that for my usecase that would be hard to mount, so I created an external 
  sensor item, connected by cable with control unit.

  
Status
------

- Circuit has passed some basic tests, ready prepared for mounting in real life

Licenses
--------

For the Schematic, Board an Spice circuit simulation files within subfolder 
``board``:

Copyright (c) 2017 by Andreas Messer. This work is licensed under the 
Creative Commons Attribution 4.0 International License. To view 
a copy of this license, visit https://creativecommons.org/licenses/by/4.0/


The model files used for Spice simulations are not included due to unclear license
regulations. You can find them somehwere in the internet.

Author
------
Andreas Messer <andi@bastelmap.de>

