EESchema Schematic File Version 2
LIBS:data-logger-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:Oscillators
LIBS:amesser_miscic
LIBS:amesser-conn
LIBS:amesser_misc
LIBS:amesser_pmic
LIBS:amesser_linear
LIBS:amesser_altera_fpga
LIBS:amesser_atmel_sam
LIBS:data-logger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 10M08DCF256-RESCUE-data-logger U?
U 4 1 586885C6
P 1700 3600
F 0 "U?" H 1700 3800 60  0000 C CNN
F 1 "10M08DCF256" H 1600 2250 60  0000 C CNN
F 2 "" H 1700 2400 60  0001 C CNN
F 3 "" H 1700 2400 60  0001 C CNN
	4    1700 3600
	1    0    0    -1  
$EndComp
$Comp
L 10M08DCF256-RESCUE-data-logger U?
U 5 1 5868865D
P 5450 2600
F 0 "U?" H 5450 2800 60  0000 C CNN
F 1 "10M08DCF256" H 5450 750 60  0000 C CNN
F 2 "" H 5450 1400 60  0001 C CNN
F 3 "" H 5450 1400 60  0001 C CNN
	5    5450 2600
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P?
U 1 1 5868FC11
P 6600 5850
F 0 "P?" H 6600 6300 50  0000 C CNN
F 1 "CONN_02X08" V 6600 5850 50  0000 C CNN
F 2 "" H 6600 4650 50  0000 C CNN
F 3 "" H 6600 4650 50  0000 C CNN
	1    6600 5850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5868FE8C
P 6450 6150
F 0 "#PWR?" H 6450 5900 50  0001 C CNN
F 1 "GND" H 6450 6000 50  0000 C CNN
F 2 "" H 6450 6150 50  0000 C CNN
F 3 "" H 6450 6150 50  0000 C CNN
	1    6450 6150
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 5868FF32
P 6450 5500
F 0 "#PWR?" H 6450 5350 50  0001 C CNN
F 1 "+3V3" H 6450 5640 50  0000 C CNN
F 2 "" H 6450 5500 50  0000 C CNN
F 3 "" H 6450 5500 50  0000 C CNN
	1    6450 5500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 586900D5
P 4650 2300
F 0 "C?" H 4675 2400 50  0000 L CNN
F 1 "4.7 µF" H 4675 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4688 2150 50  0001 C CNN
F 3 "" H 4650 2300 50  0000 C CNN
	1    4650 2300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5869022D
P 4450 2300
F 0 "C?" H 4475 2400 50  0000 L CNN
F 1 "4.7 µF" H 4475 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4488 2150 50  0001 C CNN
F 3 "" H 4450 2300 50  0000 C CNN
	1    4450 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 586902D3
P 4450 2550
F 0 "#PWR?" H 4450 2300 50  0001 C CNN
F 1 "GND" H 4450 2400 50  0000 C CNN
F 2 "" H 4450 2550 50  0000 C CNN
F 3 "" H 4450 2550 50  0000 C CNN
	1    4450 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58691008
P 900 2300
F 0 "C?" H 925 2400 50  0000 L CNN
F 1 "4.7 µF" H 925 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 938 2150 50  0001 C CNN
F 3 "" H 900 2300 50  0000 C CNN
	1    900  2300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 586910EA
P 700 2300
F 0 "C?" H 725 2400 50  0000 L CNN
F 1 "4.7 µF" H 725 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 738 2150 50  0001 C CNN
F 3 "" H 700 2300 50  0000 C CNN
	1    700  2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 586911CB
P 700 2550
F 0 "#PWR?" H 700 2300 50  0001 C CNN
F 1 "GND" H 700 2400 50  0000 C CNN
F 2 "" H 700 2550 50  0000 C CNN
F 3 "" H 700 2550 50  0000 C CNN
	1    700  2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58692DA5
P 2250 6150
F 0 "#PWR?" H 2250 5900 50  0001 C CNN
F 1 "GND" H 2250 6000 50  0000 C CNN
F 2 "" H 2250 6150 50  0000 C CNN
F 3 "" H 2250 6150 50  0000 C CNN
	1    2250 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58693228
P 3550 6150
F 0 "#PWR?" H 3550 5900 50  0001 C CNN
F 1 "GND" H 3550 6000 50  0000 C CNN
F 2 "" H 3550 6150 50  0000 C CNN
F 3 "" H 3550 6150 50  0000 C CNN
	1    3550 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58693554
P 4850 6150
F 0 "#PWR?" H 4850 5900 50  0001 C CNN
F 1 "GND" H 4850 6000 50  0000 C CNN
F 2 "" H 4850 6150 50  0000 C CNN
F 3 "" H 4850 6150 50  0000 C CNN
	1    4850 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 586957C6
P 5600 1800
F 0 "#PWR?" H 5600 1550 50  0001 C CNN
F 1 "GND" H 5600 1650 50  0000 C CNN
F 2 "" H 5600 1800 50  0000 C CNN
F 3 "" H 5600 1800 50  0000 C CNN
	1    5600 1800
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U?
U 1 1 586958B5
P 5700 1350
F 0 "U?" H 5700 1500 50  0000 L CNN
F 1 "MCP6002" H 5700 1200 50  0000 L CNN
F 2 "" H 5600 1400 50  0000 C CNN
F 3 "" H 5700 1500 50  0000 C CNN
	1    5700 1350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58695A2A
P 5800 1000
F 0 "C?" H 5825 1100 50  0000 L CNN
F 1 "100 nF" H 5825 900 50  0000 L CNN
F 2 "" H 5838 850 50  0000 C CNN
F 3 "" H 5800 1000 50  0000 C CNN
	1    5800 1000
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 58695B52
P 5600 950
F 0 "#PWR?" H 5600 800 50  0001 C CNN
F 1 "+3V3" H 5600 1090 50  0000 C CNN
F 2 "" H 5600 950 50  0000 C CNN
F 3 "" H 5600 950 50  0000 C CNN
	1    5600 950 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58695BDD
P 6000 1050
F 0 "#PWR?" H 6000 800 50  0001 C CNN
F 1 "GND" H 6000 900 50  0000 C CNN
F 2 "" H 6000 1050 50  0000 C CNN
F 3 "" H 6000 1050 50  0000 C CNN
	1    6000 1050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5869601D
P 4800 1500
F 0 "R?" V 4880 1500 50  0000 C CNN
F 1 "10 kOhm" V 4800 1500 50  0000 C CNN
F 2 "" V 4730 1500 50  0000 C CNN
F 3 "" H 4800 1500 50  0000 C CNN
	1    4800 1500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 586964BD
P 5200 1500
F 0 "R?" V 5280 1500 50  0000 C CNN
F 1 "10 kOhm" V 5200 1500 50  0000 C CNN
F 2 "" V 5130 1500 50  0000 C CNN
F 3 "" H 5200 1500 50  0000 C CNN
	1    5200 1500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 586970A9
P 6250 1350
F 0 "R?" V 6330 1350 50  0000 C CNN
F 1 "22 Ohm" V 6250 1350 50  0000 C CNN
F 2 "" V 6180 1350 50  0000 C CNN
F 3 "" H 6250 1350 50  0000 C CNN
	1    6250 1350
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 58697166
P 6450 1550
F 0 "C?" H 6475 1650 50  0000 L CNN
F 1 "4.7 µF" H 6475 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6488 1400 50  0001 C CNN
F 3 "" H 6450 1550 50  0000 C CNN
	1    6450 1550
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U?
U 2 1 58697D0E
P 1950 1350
F 0 "U?" H 1950 1500 50  0000 L CNN
F 1 "MCP6002" H 1950 1200 50  0000 L CNN
F 2 "" H 1850 1400 50  0000 C CNN
F 3 "" H 1950 1500 50  0000 C CNN
	2    1950 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58697E40
P 1850 1800
F 0 "#PWR?" H 1850 1550 50  0001 C CNN
F 1 "GND" H 1850 1650 50  0000 C CNN
F 2 "" H 1850 1800 50  0000 C CNN
F 3 "" H 1850 1800 50  0000 C CNN
	1    1850 1800
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P?
U 1 1 58691A75
P 2400 5850
F 0 "P?" H 2400 6300 50  0000 C CNN
F 1 "CONN_02X08" V 2400 5850 50  0000 C CNN
F 2 "" H 2400 4650 50  0000 C CNN
F 3 "" H 2400 4650 50  0000 C CNN
	1    2400 5850
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 58691CC3
P 2250 5500
F 0 "#PWR?" H 2250 5350 50  0001 C CNN
F 1 "+3V3" H 2250 5640 50  0000 C CNN
F 2 "" H 2250 5500 50  0000 C CNN
F 3 "" H 2250 5500 50  0000 C CNN
	1    2250 5500
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P?
U 1 1 5869218E
P 3700 5850
F 0 "P?" H 3700 6300 50  0000 C CNN
F 1 "CONN_02X08" V 3700 5850 50  0000 C CNN
F 2 "" H 3700 4650 50  0000 C CNN
F 3 "" H 3700 4650 50  0000 C CNN
	1    3700 5850
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 58692340
P 3550 5500
F 0 "#PWR?" H 3550 5350 50  0001 C CNN
F 1 "+3V3" H 3550 5640 50  0000 C CNN
F 2 "" H 3550 5500 50  0000 C CNN
F 3 "" H 3550 5500 50  0000 C CNN
	1    3550 5500
	1    0    0    -1  
$EndComp
NoConn ~ 2150 5600
$Comp
L CONN_02X08 P?
U 1 1 5869259B
P 5000 5850
F 0 "P?" H 5000 6300 50  0000 C CNN
F 1 "CONN_02X08" V 5000 5850 50  0000 C CNN
F 2 "" H 5000 4650 50  0000 C CNN
F 3 "" H 5000 4650 50  0000 C CNN
	1    5000 5850
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 58692613
P 4850 5500
F 0 "#PWR?" H 4850 5350 50  0001 C CNN
F 1 "+3V3" H 4850 5640 50  0000 C CNN
F 2 "" H 4850 5500 50  0000 C CNN
F 3 "" H 4850 5500 50  0000 C CNN
	1    4850 5500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5869286E
P 1450 1500
F 0 "R?" V 1530 1500 50  0000 C CNN
F 1 "10 kOhm" V 1450 1500 50  0000 C CNN
F 2 "" V 1380 1500 50  0000 C CNN
F 3 "" H 1450 1500 50  0000 C CNN
	1    1450 1500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58692C4F
P 2500 1350
F 0 "R?" V 2580 1350 50  0000 C CNN
F 1 "22 Ohm" V 2500 1350 50  0000 C CNN
F 2 "" V 2430 1350 50  0000 C CNN
F 3 "" H 2500 1350 50  0000 C CNN
	1    2500 1350
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 58692CEA
P 2700 1550
F 0 "C?" H 2725 1650 50  0000 L CNN
F 1 "4.7 µF" H 2725 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 2738 1400 50  0001 C CNN
F 3 "" H 2700 1550 50  0000 C CNN
	1    2700 1550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5869385E
P 1050 1500
F 0 "R?" V 1130 1500 50  0000 C CNN
F 1 "10 kOhm" V 1050 1500 50  0000 C CNN
F 2 "" V 980 1500 50  0000 C CNN
F 3 "" H 1050 1500 50  0000 C CNN
	1    1050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 6100 6450 6150
Wire Wire Line
	4800 2200 4850 2200
Wire Wire Line
	4450 2100 4850 2100
Connection ~ 4800 2200
Wire Wire Line
	6250 2200 6250 5600
Wire Wire Line
	4650 2100 4650 2150
Connection ~ 4800 2100
Wire Wire Line
	4450 2100 4450 2150
Connection ~ 4650 2100
Wire Wire Line
	4450 2500 4650 2500
Wire Wire Line
	4650 2500 4650 2450
Wire Wire Line
	4450 2450 4450 2550
Connection ~ 4450 2500
Wire Wire Line
	6950 6100 7050 6100
Wire Wire Line
	7050 6100 7050 3200
Wire Wire Line
	7050 3200 6050 3200
Wire Wire Line
	6050 3000 7100 3000
Wire Wire Line
	7100 3000 7100 6200
Wire Wire Line
	7100 6200 6850 6200
Wire Wire Line
	6850 6200 6850 6100
Wire Wire Line
	6050 2800 7150 2800
Wire Wire Line
	7150 2800 7150 6250
Wire Wire Line
	7150 6250 6750 6250
Wire Wire Line
	6750 6250 6750 6100
Wire Wire Line
	6050 2600 7200 2600
Wire Wire Line
	7200 2600 7200 6300
Wire Wire Line
	7200 6300 6650 6300
Wire Wire Line
	6650 6300 6650 6100
Wire Wire Line
	6050 2400 7250 2400
Wire Wire Line
	7250 2400 7250 6350
Wire Wire Line
	7250 6350 6550 6350
Wire Wire Line
	6550 6350 6550 6100
Wire Wire Line
	6550 5600 6550 2300
Wire Wire Line
	6550 2300 6050 2300
Wire Wire Line
	6650 5600 6650 2500
Wire Wire Line
	6650 2500 6050 2500
Wire Wire Line
	6750 2700 6750 5600
Wire Wire Line
	6750 2700 6050 2700
Wire Wire Line
	6850 5600 6850 2900
Wire Wire Line
	6850 2900 6050 2900
Wire Wire Line
	6950 5600 6950 3100
Wire Wire Line
	6950 3100 6050 3100
Wire Wire Line
	700  2100 1100 2100
Wire Wire Line
	1050 2300 1100 2300
Wire Wire Line
	1050 2200 1100 2200
Connection ~ 1050 2200
Wire Wire Line
	700  2450 700  2550
Wire Wire Line
	700  2500 900  2500
Wire Wire Line
	900  2500 900  2450
Connection ~ 700  2500
Connection ~ 1050 2100
Wire Wire Line
	900  2150 900  2100
Connection ~ 900  2100
Wire Wire Line
	1050 5300 3450 5300
Connection ~ 1050 2300
Wire Wire Line
	2350 5600 2350 2300
Wire Wire Line
	2350 2300 2300 2300
Wire Wire Line
	2450 5600 2450 2500
Wire Wire Line
	2450 2500 2300 2500
Wire Wire Line
	2550 5600 2550 2700
Wire Wire Line
	2550 2700 2300 2700
Wire Wire Line
	2650 5600 2650 2900
Wire Wire Line
	2650 2900 2300 2900
Wire Wire Line
	2750 5600 2750 3100
Wire Wire Line
	2750 3100 2300 3100
Wire Wire Line
	2300 3200 2850 3200
Wire Wire Line
	2850 3200 2850 6150
Wire Wire Line
	2850 6150 2750 6150
Wire Wire Line
	2750 6150 2750 6100
Wire Wire Line
	2650 6100 2650 6200
Wire Wire Line
	2650 6200 2900 6200
Wire Wire Line
	2900 6200 2900 3000
Wire Wire Line
	2900 3000 2300 3000
Wire Wire Line
	2300 2800 2950 2800
Wire Wire Line
	2950 2800 2950 6250
Wire Wire Line
	2950 6250 2550 6250
Wire Wire Line
	2550 6250 2550 6100
Wire Wire Line
	2450 6100 2450 6300
Wire Wire Line
	2450 6300 3000 6300
Wire Wire Line
	3000 6300 3000 2600
Wire Wire Line
	3000 2600 2300 2600
Wire Wire Line
	2300 2400 3050 2400
Wire Wire Line
	3050 2400 3050 6350
Wire Wire Line
	3050 6350 2350 6350
Wire Wire Line
	2350 6350 2350 6100
Wire Wire Line
	2250 6100 2250 6150
Wire Wire Line
	4150 6150 4050 6150
Wire Wire Line
	4050 6150 4050 6100
Wire Wire Line
	3950 6100 3950 6200
Wire Wire Line
	3950 6200 4200 6200
Wire Wire Line
	4250 6250 3850 6250
Wire Wire Line
	3850 6250 3850 6100
Wire Wire Line
	3750 6100 3750 6300
Wire Wire Line
	3750 6300 4300 6300
Wire Wire Line
	4350 6350 3650 6350
Wire Wire Line
	3650 6350 3650 6100
Wire Wire Line
	3550 6100 3550 6150
Wire Wire Line
	5450 6150 5350 6150
Wire Wire Line
	5350 6150 5350 6100
Wire Wire Line
	5250 6100 5250 6200
Wire Wire Line
	5250 6200 5500 6200
Wire Wire Line
	5550 6250 5150 6250
Wire Wire Line
	5150 6250 5150 6100
Wire Wire Line
	5050 6100 5050 6300
Wire Wire Line
	5050 6300 5600 6300
Wire Wire Line
	5650 6350 4950 6350
Wire Wire Line
	4950 6350 4950 6100
Wire Wire Line
	4850 6100 4850 6150
Wire Wire Line
	3650 5600 3650 3300
Wire Wire Line
	3650 3300 2300 3300
Wire Wire Line
	2300 3500 3750 3500
Wire Wire Line
	3750 3500 3750 5600
Wire Wire Line
	3850 5600 3850 3700
Wire Wire Line
	3850 3700 2300 3700
Wire Wire Line
	2300 3900 3950 3900
Wire Wire Line
	3950 3900 3950 5600
Wire Wire Line
	2300 4100 4050 4100
Wire Wire Line
	4050 4100 4050 5600
Wire Wire Line
	4150 6150 4150 4200
Wire Wire Line
	4150 4200 2300 4200
Wire Wire Line
	2300 4000 4200 4000
Wire Wire Line
	4200 4000 4200 6200
Wire Wire Line
	4250 6250 4250 3800
Wire Wire Line
	4250 3800 2300 3800
Wire Wire Line
	2300 3600 4300 3600
Wire Wire Line
	4300 3600 4300 6300
Wire Wire Line
	4350 6350 4350 3400
Wire Wire Line
	4350 3400 2300 3400
Wire Wire Line
	4950 5600 4950 4300
Wire Wire Line
	4950 4300 2300 4300
Wire Wire Line
	2300 4500 5050 4500
Wire Wire Line
	5050 4500 5050 5600
Wire Wire Line
	5150 5600 5150 4700
Wire Wire Line
	5150 4700 2300 4700
Wire Wire Line
	2300 4900 5250 4900
Wire Wire Line
	5250 4900 5250 5600
Wire Wire Line
	5350 5100 5350 5600
Wire Wire Line
	5350 5100 2300 5100
Wire Wire Line
	5450 6150 5450 5200
Wire Wire Line
	5450 5200 2300 5200
Wire Wire Line
	5500 6200 5500 5000
Wire Wire Line
	5500 5000 2300 5000
Wire Wire Line
	5550 6250 5550 4800
Wire Wire Line
	5550 4800 2300 4800
Wire Wire Line
	5600 6300 5600 4600
Wire Wire Line
	5600 4600 2300 4600
Wire Wire Line
	5650 6350 5650 4400
Wire Wire Line
	5650 4400 2300 4400
Wire Wire Line
	5600 950  5600 1050
Wire Wire Line
	5600 1000 5650 1000
Connection ~ 5600 1000
Wire Wire Line
	5950 1000 6000 1000
Wire Wire Line
	6000 1000 6000 1050
Wire Wire Line
	6000 1350 6100 1350
Wire Wire Line
	5600 1650 5600 1800
Wire Wire Line
	5400 1450 5350 1450
Wire Wire Line
	5350 1450 5350 1700
Wire Wire Line
	5350 1700 6050 1700
Wire Wire Line
	6050 1700 6050 1350
Connection ~ 6050 1350
Wire Wire Line
	5000 1750 6450 1750
Wire Wire Line
	5200 1750 5200 1650
Connection ~ 5600 1750
Wire Wire Line
	4800 1250 5400 1250
Connection ~ 5200 1250
Wire Wire Line
	6450 1750 6450 1700
Wire Wire Line
	6400 1350 6600 1350
Wire Wire Line
	6450 1350 6450 1400
Wire Wire Line
	6600 1350 6600 2100
Wire Wire Line
	6600 2100 6050 2100
Connection ~ 6450 1350
Wire Wire Line
	1850 1650 1850 1800
Wire Wire Line
	1650 1450 1600 1450
Wire Wire Line
	1600 1450 1600 1700
Wire Wire Line
	1600 1700 2300 1700
Wire Wire Line
	2300 1700 2300 1350
Wire Wire Line
	2250 1350 2350 1350
Wire Wire Line
	1050 1250 1650 1250
Wire Wire Line
	6350 3300 6350 5600
Wire Wire Line
	6450 5500 6450 5600
Wire Wire Line
	6250 2200 6050 2200
Wire Wire Line
	2250 5500 2250 5600
Wire Wire Line
	3450 5300 3450 5600
Wire Wire Line
	3550 5500 3550 5600
Wire Wire Line
	4850 5500 4850 5600
Wire Wire Line
	1250 1750 2700 1750
Wire Wire Line
	1450 1750 1450 1650
Connection ~ 1850 1750
Wire Wire Line
	1450 1250 1450 1350
Wire Wire Line
	5200 1250 5200 1350
Wire Wire Line
	2700 1750 2700 1700
Connection ~ 2300 1350
Wire Wire Line
	2650 1350 2850 1350
Wire Wire Line
	2700 1350 2700 1400
Connection ~ 1450 1250
Wire Wire Line
	700  2100 700  2150
Wire Wire Line
	2850 1350 2850 2100
Wire Wire Line
	2850 2100 2300 2100
Connection ~ 2700 1350
Wire Wire Line
	6250 6400 6250 6100
Wire Wire Line
	6350 6450 6350 6100
$Comp
L +1V8 #PWR?
U 1 1 586953E8
P 1850 6250
F 0 "#PWR?" H 1850 6100 50  0001 C CNN
F 1 "+1V8" H 1850 6390 50  0000 C CNN
F 2 "" H 1850 6250 50  0000 C CNN
F 3 "" H 1850 6250 50  0000 C CNN
	1    1850 6250
	1    0    0    -1  
$EndComp
$Comp
L +1V2 #PWR?
U 1 1 5869576F
P 1650 6250
F 0 "#PWR?" H 1650 6100 50  0001 C CNN
F 1 "+1V2" H 1650 6390 50  0000 C CNN
F 2 "" H 1650 6250 50  0000 C CNN
F 3 "" H 1650 6250 50  0000 C CNN
	1    1650 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 6400 6250 6400
Wire Wire Line
	3350 6400 3350 6100
Wire Wire Line
	4650 6400 4650 6100
Connection ~ 3350 6400
Wire Wire Line
	1850 6250 1850 6400
Connection ~ 2050 6400
Wire Wire Line
	1650 6250 1650 6450
Wire Wire Line
	1650 6450 6350 6450
Wire Wire Line
	2150 6450 2150 6100
Wire Wire Line
	3450 6450 3450 6100
Connection ~ 2150 6450
Wire Wire Line
	4750 6450 4750 6100
Connection ~ 3450 6450
Connection ~ 4650 6400
Connection ~ 4750 6450
Wire Wire Line
	2050 6100 2050 6400
NoConn ~ 4750 5600
Wire Wire Line
	2050 5600 2050 5550
Wire Wire Line
	2000 5550 4650 5550
Wire Wire Line
	3350 2200 3350 5600
Wire Wire Line
	4650 5550 4650 5600
Connection ~ 3350 5550
Wire Wire Line
	3350 2200 2300 2200
$Comp
L R R?
U 1 1 58697DCC
P 1850 5550
F 0 "R?" V 1930 5550 50  0000 C CNN
F 1 "10 kOhm" V 1850 5550 50  0000 C CNN
F 2 "" V 1780 5550 50  0000 C CNN
F 3 "" H 1850 5550 50  0000 C CNN
	1    1850 5550
	0    1    1    0   
$EndComp
Connection ~ 2050 5550
Wire Wire Line
	1700 5550 1650 5550
Wire Wire Line
	1650 5550 1650 5300
Connection ~ 1650 5300
$Comp
L R R?
U 1 1 58698362
P 6050 3550
F 0 "R?" V 6130 3550 50  0000 C CNN
F 1 "10 kOhm" V 6050 3550 50  0000 C CNN
F 2 "" V 5980 3550 50  0000 C CNN
F 3 "" H 6050 3550 50  0000 C CNN
	1    6050 3550
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 586990A7
P 5000 1500
F 0 "C?" H 5025 1600 50  0000 L CNN
F 1 "100 nF" H 5025 1400 50  0000 L CNN
F 2 "" H 5038 1350 50  0000 C CNN
F 3 "" H 5000 1500 50  0000 C CNN
	1    5000 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 1250 4800 1350
Wire Wire Line
	5000 1350 5000 1250
Connection ~ 5000 1250
Wire Wire Line
	5000 1650 5000 1750
Connection ~ 5200 1750
$Comp
L C C?
U 1 1 5869970B
P 1250 1500
F 0 "C?" H 1275 1600 50  0000 L CNN
F 1 "100 nF" H 1275 1400 50  0000 L CNN
F 2 "" H 1288 1350 50  0000 C CNN
F 3 "" H 1250 1500 50  0000 C CNN
	1    1250 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 1350 1050 1250
Wire Wire Line
	1250 1250 1250 1350
Connection ~ 1250 1250
Wire Wire Line
	1250 1650 1250 1750
Connection ~ 1450 1750
Wire Wire Line
	1050 1650 1050 5300
Wire Wire Line
	4800 3300 6350 3300
Wire Wire Line
	4800 1650 4800 3300
Connection ~ 5850 3300
Wire Wire Line
	5850 3300 5850 3550
Wire Wire Line
	5850 3550 5900 3550
Wire Wire Line
	6200 3550 6350 3550
Connection ~ 6350 3550
$EndSCHEMATC
