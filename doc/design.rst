
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
-------------------

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
      | USB           |      |   00 |      |      |        |      | 
      | Clock         |      |      |      |    x |        |
Clock | CDCE          |      |      |      |   15 |    40  | 
------+---------------+------+------+------+------+--------+------+-----
Sum   |               | 2000 |  200 |  800 |   50 |   800  |  200 | 60

Decoupling Caps
```````````````

- 1.1V  : 15-20x 4.7µF, 5x 10µF
- 1.35V : 6x 4.7µF
- 2.5V  : 10x 4.7µF, 2x 10µF
- 3.3V  :  7x 4.7µF


Power design topology
---------------------

- Input -> 3.3V: Rail + 0.3A  (7.3W + 1W) Out, ~9W In, TPS53015 + 2xCSD17313Q2 (max 32mOhm Rds_on)
- 3.3V  -> 1.1V: 2A        2.2W Out, 2.6W In (87%): TLV62085
- 3.3V  -> 1.35V: 0.8A     1.1W Out, 1.2W In (90%): TLV62085
- 3.3V  -> 1.2V: 0.3A     0.4W Out, 0.5W In (85%): MIC23050
- 3.3V  -> 2.5V: 1A       (2.5W + 0.3W) Out, 3.0W In (95%): TLV62085
- 2.5V  -> 1.8V: LDO 0.1A 0.2W Out, 0.3W In

Vbus to Vin
+++++++++++

Using NCP373 to prevent reverse current flow into usb jacket and to limit usb current. Current limit resistor values:

- 348k for 100mA  (USB, plugged)
- 47k  for 500mA  (USB, configured)
- 12.8k for 900mA (USB3.0)


Vin to 3.3V Vsys
++++++++++++++++++

Made using TPS53015 and external mosfets to achieve high efficiency but also 
appropiate current limit. Used Design parameters:

Vin: 4.5V to 24V
Vout: 3.3V
Iout(max): 3.0A
Iripple:   0.9A
Vout(Ripple): 30mV
fsw: 480kHz
Rdson: 26 - 38mOhm
VTrip: 87 mV (~3A Current Limit)
Qg: 2.5nC@5V

According datasheet the following is calculated:

Ton: 11µs - 286ns
Lout: 6.8µH
IL(Ripple): 0.9A
IL(Peak): 4.5A
Cout: 10µF
Cout(Overshot, 3A Transient, 100mV): 100µF
Cout(Undershot, 3A Transient, 100mV): 220µF

Cbs(2.5nC, 480kHz): 100nF 

Vsys to 3.3V rail
+++++++++++++++++

Using NCP380HMUAJAATBG current limiting power distribution switch. Use 20mOhm coils
to optimize efficiency.
 
Vsys to 2.5V, 1.35V, 1.1V rail
++++++++++++++++++++++++++++++

Using TLV62085 DC/DC converters, calculated using Ti WebBench

NCP373


