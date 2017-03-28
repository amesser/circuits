
Hardware Design
===============

Parts
-----

Altera Cyclone V FPGA
`````````````````````

Required IO Resources
  - Cypress FX3 Interface: 44 i/o pins
  - DDR3 RAM Interface
  - User IO Ports: 3x16 i/o pins with configurable io voltage
  - Extra IO Port for Leds, Display, Buttons: 16 i/o pins, fixed to 3.3V

Device
  - 5CEFA2F23 in FBGA486 Package, 1.00mm pitch

Power Supply Requirements
  Simulated with quartus and example project
  - 1.1V/140mA Core +-30mV
  - 1.35V/80mA IO
  - 2.5V/1mA VCCPGM
  - 2.5V/75mA IO PD
  - 2.5V/10mA PLL
  - 2.5V/70mA VCCAUX
 
IS43TR16256AL-15HBL
------------------

512MB DDR3 SDRAM

Package
  - 96-BGA, 0.80mm

Power Supply
  - 1.35V 3%/250 mA

CYUSB3012-BZXC
--------------

Provides USB3.1 connection to host PC. Interfaces with FPGA using
32 Bit GPIF interface.


IO Pins
  - FPGA Interface 32 data + 11 control + 1 clock line 

Power supply
  - 1.2V/200mA core 
  - 2.5V IO to FPGA
  - 1.8V Clock
  - 1.2V/60mA USB3.0
  - 3.3V USB2.0 (internal voltage regulator)

Clocking
--------

CDCE913
  - 1.8V Powersupply
  - 2.5/3.3 V Output
  - 3 Clocks from one crystal
  - Default startup configuration

Measurement Adapters
--------------------

ADC Adapter
~~~~~~~~~~~

LTC2325-16 4 Channel 16 Bit 5MSPS
  - 3.3 or 5V Analog Powersupply
  - 1.8 to 2.5 V Digital Power Supply

Power Supply
------------

NCP51200
  - 3A DDR VTT Regulator
  - 2.5V VCC 1mA
  - 1.35V DDR Powersupply


Power Supply
----------------

Power requirements
``````````````````
      |               | 1.10 | 1.20 | 1.35 | 1.80 |  2.50  | 3.30 | VUSB
FPGA  | Core (VDD)    |  180 |      |      |      |        | 
      | PLL           |      |      |      |      |    15  |
      | JTAG/PGM      |      |      |      |      |     1  |
      | BAT           |      |    0 |      |      |        |
      | AUX           |      |      |      |      |    65  |
      | IO DDR3       |      |      |   75 |      |    70  | 
      | IO FX3        |      |      |      |      |    50  | 
      | IO USER       |      |      |      |      |   150  |  150
      | IO EXT        |      |      |      |      |    20  |   50
      | (Peak)        |      |      |      |      | (1000) |
DDR3  | RAM           |      |      |  250 |      |        |
      | VTT           |      |      |  350 |      |     1  |
      | Clock         |      |      |      |      |    30  |
FX3   | Core + Analog |      |  200 |      |      |        |
      | IO            |      |      |      |      |   100  |
      | USB           |      |      |      |      |        |      | 60
      | Clock         |      |      |      |    x |        |
Clock | CDCE          |      |      |      |   15 |    40  | 
------+---------------+------+------+------+------+--------+------+-----
Sum   |               | 2000 |  250 |  800 |   50 |   800  |  200 | 60

Decoupling Caps
```````````````

- 1.1V  : 15-20x 4.7µF, 5x 10µF
- 1.35V : 6x 4.7µF
- 2.5V  : 10x 4.7µF, 2x 10µF
- 3.3V  :  7x 4.7µF


MIC23050: 600mA, 4MHz: 1.2V, 3.3V
NCP6332B: 1.2A, 3MHz: 1.35V, 2.5V
NCP6324B: 2A, 3MHz: 1.1V



