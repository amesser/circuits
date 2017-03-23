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
LIBS:amesser_memory
LIBS:amesser_cypress_fx
LIBS:amesser_osc_crystal
LIBS:amesser_diodes
LIBS:data-logger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 9 9
Title "Altera MAX 10 FPGA Development Board"
Date "2017-01-14"
Rev "1"
Comp "Copyright (c) 2017 Andreas Messer"
Comment1 "USB, JTAG, SD-CARD and miscellanous stuff "
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CYUSB3012-BZXC U?
U 3 1 58D4ADBE
P 2600 1850
F 0 "U?" H 2200 2650 60  0000 C CNN
F 1 "CYUSB3012-BZXC" H 2600 950 60  0000 C CNN
F 2 "" H 2600 1450 60  0001 C CNN
F 3 "" H 2600 1450 60  0001 C CNN
	3    2600 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4AE61
P 1450 3650
F 0 "C?" H 1475 3750 50  0000 L CNN
F 1 "4µ7" H 1250 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1488 3500 50  0001 C CNN
F 3 "" H 1450 3650 50  0000 C CNN
	1    1450 3650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4AEF4
P 1550 1850
F 0 "C?" H 1575 1950 50  0000 L CNN
F 1 "10µ" H 1575 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1588 1700 50  0001 C CNN
F 3 "" H 1550 1850 50  0000 C CNN
	1    1550 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4AF48
P 1350 1850
F 0 "C?" H 1375 1950 50  0000 L CNN
F 1 "10µ" H 1375 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1388 1700 50  0001 C CNN
F 3 "" H 1350 1850 50  0000 C CNN
	1    1350 1850
	1    0    0    -1  
$EndComp
$Comp
L L_Small L?
U 1 1 58D4BA86
P 1200 1650
F 0 "L?" V 1300 1700 50  0000 R CNN
F 1 "50Ohm@100MHz" V 1250 1700 50  0000 R CNN
F 2 "" H 1200 1650 50  0000 C CNN
F 3 "" H 1200 1650 50  0000 C CNN
	1    1200 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 1650 1900 1650
Wire Wire Line
	1750 1650 1750 1700
Connection ~ 1750 1650
Wire Wire Line
	1550 1650 1550 1700
Connection ~ 1550 1650
Wire Wire Line
	1350 1650 1350 1700
Connection ~ 1350 1650
Wire Wire Line
	1750 2050 1750 2000
Wire Wire Line
	1350 2050 1750 2050
Wire Wire Line
	1550 2000 1550 2100
Wire Wire Line
	1350 2000 1350 2050
Connection ~ 1550 2050
$Comp
L GND #PWR?
U 1 1 58D4BC20
P 1550 2100
F 0 "#PWR?" H 1550 1850 50  0001 C CNN
F 1 "GND" H 1550 1950 50  0001 C CNN
F 2 "" H 1550 2100 50  0000 C CNN
F 3 "" H 1550 2100 50  0000 C CNN
	1    1550 2100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4CB1A
P 1750 2550
F 0 "C?" H 1775 2650 50  0000 L CNN
F 1 "100n" H 1775 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1788 2400 50  0001 C CNN
F 3 "" H 1750 2550 50  0000 C CNN
	1    1750 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4CB20
P 1550 2550
F 0 "C?" H 1575 2650 50  0000 L CNN
F 1 "10µ" H 1575 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1588 2400 50  0001 C CNN
F 3 "" H 1550 2550 50  0000 C CNN
	1    1550 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D4CB26
P 1350 2550
F 0 "C?" H 1375 2650 50  0000 L CNN
F 1 "10µ" H 1375 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1388 2400 50  0001 C CNN
F 3 "" H 1350 2550 50  0000 C CNN
	1    1350 2550
	1    0    0    -1  
$EndComp
$Comp
L L_Small L?
U 1 1 58D4CB2C
P 1200 2350
F 0 "L?" V 1300 2400 50  0000 R CNN
F 1 "50Ohm@100MHz" V 1250 2400 50  0000 R CNN
F 2 "" H 1200 2350 50  0000 C CNN
F 3 "" H 1200 2350 50  0000 C CNN
	1    1200 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 2350 1900 2350
Wire Wire Line
	1750 2350 1750 2400
Connection ~ 1750 2350
Wire Wire Line
	1550 2350 1550 2400
Connection ~ 1550 2350
Wire Wire Line
	1350 2350 1350 2400
Connection ~ 1350 2350
Wire Wire Line
	1750 2750 1750 2700
Wire Wire Line
	1350 2750 1750 2750
Wire Wire Line
	1550 2700 1550 2800
Wire Wire Line
	1350 2700 1350 2750
Connection ~ 1550 2750
$Comp
L GND #PWR?
U 1 1 58D4CB3E
P 1550 2800
F 0 "#PWR?" H 1550 2550 50  0001 C CNN
F 1 "GND" H 1550 2650 50  0001 C CNN
F 2 "" H 1550 2800 50  0000 C CNN
F 3 "" H 1550 2800 50  0000 C CNN
	1    1550 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  1650 1100 1650
Wire Wire Line
	1050 1650 1050 2350
Wire Wire Line
	1050 2350 1100 2350
Text Label 650  1650 0    60   ~ 0
VCC_1V20
Connection ~ 1050 1650
$Comp
L R R?
U 1 1 58D4DFC8
P 3500 2250
F 0 "R?" V 3500 2250 50  0000 C CNN
F 1 "6k04 1%" V 3500 1850 50  0000 C CNN
F 2 "" V 3430 2250 50  0000 C CNN
F 3 "" H 3500 2250 50  0000 C CNN
	1    3500 2250
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 58D4E03C
P 3500 2350
F 0 "R?" V 3500 2350 50  0000 C CNN
F 1 "200R 1%" V 3500 1950 50  0000 C CNN
F 2 "" V 3430 2350 50  0000 C CNN
F 3 "" H 3500 2350 50  0000 C CNN
	1    3500 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 2250 3350 2250
Wire Wire Line
	3300 2350 3350 2350
$Comp
L GND #PWR?
U 1 1 58D4E11D
P 3700 2400
F 0 "#PWR?" H 3700 2150 50  0001 C CNN
F 1 "GND" H 3700 2250 50  0001 C CNN
F 2 "" H 3700 2400 50  0000 C CNN
F 3 "" H 3700 2400 50  0000 C CNN
	1    3700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2350 3700 2350
Wire Wire Line
	3700 2250 3700 2400
Wire Wire Line
	3650 2250 3700 2250
Connection ~ 3700 2350
$Comp
L C C?
U 1 1 58D4EC64
P 3500 1650
F 0 "C?" H 3525 1750 50  0000 L CNN
F 1 "100n" H 3525 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3538 1500 50  0001 C CNN
F 3 "" H 3500 1650 50  0000 C CNN
	1    3500 1650
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 58D4ED24
P 3500 1850
F 0 "C?" H 3525 1950 50  0000 L CNN
F 1 "100n" H 3525 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3538 1700 50  0001 C CNN
F 3 "" H 3500 1850 50  0000 C CNN
	1    3500 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 1650 3350 1650
Wire Wire Line
	3300 1850 3350 1850
$Comp
L SP3010-04UTG U?
U 1 1 58D4F8F0
P 4650 1550
F 0 "U?" H 4500 1850 60  0000 C CNN
F 1 "SP3010-04UTG" H 4650 1250 60  0000 C CNN
F 2 "" H 4650 1350 60  0001 C CNN
F 3 "" H 4650 1350 60  0001 C CNN
	1    4650 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1750 4300 1750
Wire Wire Line
	4300 1750 4300 1950
Wire Wire Line
	4300 1950 5000 1950
Wire Wire Line
	5000 1950 5000 1750
Wire Wire Line
	5000 1750 4950 1750
$Comp
L GND #PWR?
U 1 1 58D4FA2C
P 4650 2000
F 0 "#PWR?" H 4650 1750 50  0001 C CNN
F 1 "GND" H 4650 1850 50  0001 C CNN
F 2 "" H 4650 2000 50  0000 C CNN
F 3 "" H 4650 2000 50  0000 C CNN
	1    4650 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1950 4650 2000
Connection ~ 4650 1950
Wire Wire Line
	3700 1550 4350 1550
Wire Wire Line
	3700 1650 3650 1650
Wire Wire Line
	3650 1850 3750 1850
Wire Wire Line
	3750 1650 4350 1650
Text Label 3750 1350 0    60   ~ 0
USB_SS_RX_N
Text Label 3750 1450 0    60   ~ 0
USB_SS_RX_P
Text Label 3750 1550 0    60   ~ 0
USB_SS_TX_N
Text Label 3750 1650 0    60   ~ 0
USB_SS_TX_P
Text Label 4950 1350 0    60   ~ 0
USB_SS_RX_N
Text Label 4950 1450 0    60   ~ 0
USB_SS_RX_P
Text Label 4950 1550 0    60   ~ 0
USB_SS_TX_N
Text Label 4950 1650 0    60   ~ 0
USB_SS_TX_P
$Comp
L USB_B_3 P?
U 1 1 58D50F25
P 6150 1350
F 0 "P?" H 5900 1950 50  0000 C CNN
F 1 "USB_B_3" H 6150 850 50  0000 C CNN
F 2 "" H 6250 1450 50  0000 C CNN
F 3 "" H 6250 1450 50  0000 C CNN
	1    6150 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1450 4350 1450
Wire Wire Line
	3700 1550 3700 1650
Wire Wire Line
	3750 1850 3750 1650
Wire Wire Line
	3300 1250 3700 1250
Wire Wire Line
	3700 1250 3700 1350
Wire Wire Line
	3700 1350 4350 1350
Wire Wire Line
	5650 1350 4950 1350
Wire Wire Line
	5650 1450 4950 1450
Wire Wire Line
	5650 1550 4950 1550
Wire Wire Line
	5650 1650 4950 1650
Wire Wire Line
	3300 950  5650 950 
Wire Wire Line
	5650 1050 3300 1050
$Comp
L GND #PWR?
U 1 1 58D512A0
P 5600 1800
F 0 "#PWR?" H 5600 1550 50  0001 C CNN
F 1 "GND" H 5600 1650 50  0001 C CNN
F 2 "" H 5600 1800 50  0000 C CNN
F 3 "" H 5600 1800 50  0000 C CNN
	1    5600 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1150 5600 1150
Wire Wire Line
	5600 1150 5600 1800
Wire Wire Line
	5600 1750 5650 1750
Connection ~ 5600 1750
$Comp
L R R?
U 1 1 58D5148C
P 6900 1050
F 0 "R?" V 6980 1050 50  0000 C CNN
F 1 "1M" V 6900 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 6830 1050 50  0001 C CNN
F 3 "" H 6900 1050 50  0000 C CNN
	1    6900 1050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D514D3
P 6700 1050
F 0 "C?" H 6725 1150 50  0000 L CNN
F 1 "100n" H 6725 950 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6738 900 50  0001 C CNN
F 3 "" H 6700 1050 50  0000 C CNN
	1    6700 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 850  6900 850 
Wire Wire Line
	6900 850  6900 900 
Wire Wire Line
	6700 850  6700 900 
Connection ~ 6700 850 
Wire Wire Line
	6700 1200 6700 1300
Wire Wire Line
	6700 1250 6900 1250
Wire Wire Line
	6900 1250 6900 1200
$Comp
L GND #PWR?
U 1 1 58D5188D
P 6700 1300
F 0 "#PWR?" H 6700 1050 50  0001 C CNN
F 1 "GND" H 6700 1150 50  0001 C CNN
F 2 "" H 6700 1300 50  0000 C CNN
F 3 "" H 6700 1300 50  0000 C CNN
	1    6700 1300
	1    0    0    -1  
$EndComp
Connection ~ 6700 1250
Text GLabel 6500 650  2    60   Output ~ 0
VCC_USB
Wire Wire Line
	5600 850  5650 850 
Wire Wire Line
	650  950  1900 950 
$Comp
L C C?
U 1 1 58D53188
P 1750 1150
F 0 "C?" H 1775 1250 50  0000 L CNN
F 1 "100n" H 1775 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1788 1000 50  0001 C CNN
F 3 "" H 1750 1150 50  0000 C CNN
	1    1750 1150
	1    0    0    -1  
$EndComp
Text Label 650  850  0    60   ~ 0
VCC_3V30
Wire Wire Line
	650  850  1900 850 
Text Label 650  950  0    60   ~ 0
VCC_USB
$Comp
L C C?
U 1 1 58D53FBA
P 1550 1150
F 0 "C?" H 1575 1250 50  0000 L CNN
F 1 "100n" H 1575 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1588 1000 50  0001 C CNN
F 3 "" H 1550 1150 50  0000 C CNN
	1    1550 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1000 1550 850 
Connection ~ 1550 850 
Wire Wire Line
	1750 1000 1750 950 
Connection ~ 1750 950 
Wire Wire Line
	1550 1300 1550 1400
Wire Wire Line
	1550 1350 1750 1350
Wire Wire Line
	1750 1350 1750 1300
$Comp
L GND #PWR?
U 1 1 58D541BD
P 1550 1400
F 0 "#PWR?" H 1550 1150 50  0001 C CNN
F 1 "GND" H 1550 1250 50  0001 C CNN
F 2 "" H 1550 1400 50  0000 C CNN
F 3 "" H 1550 1400 50  0000 C CNN
	1    1550 1400
	1    0    0    -1  
$EndComp
Connection ~ 1550 1350
$Comp
L L_Small L?
U 1 1 58D567AE
P 5750 650
F 0 "L?" V 5850 700 50  0000 R CNN
F 1 "50Ohm@100MHz" V 5800 950 50  0000 R CNN
F 2 "" H 5750 650 50  0000 C CNN
F 3 "" H 5750 650 50  0000 C CNN
	1    5750 650 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5600 850  5600 650 
Wire Wire Line
	5600 650  5650 650 
Wire Wire Line
	5850 650  6500 650 
Text Label 6400 650  2    60   ~ 0
VCC_USB
$Comp
L 5CEFA2F23 U?
U 2 1 58D588FB
P 2500 4150
F 0 "U?" H 2000 4950 60  0000 C CNN
F 1 "5CEFA2F23" H 2500 3250 60  0000 C CNN
F 2 "" H 2500 4250 60  0001 C CNN
F 3 "" H 2500 4250 60  0001 C CNN
	2    2500 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3450 1700 3450
Wire Wire Line
	1450 3450 1450 3500
Wire Wire Line
	1650 3450 1650 3750
Wire Wire Line
	1650 3550 1700 3550
Connection ~ 1650 3450
Wire Wire Line
	1650 3650 1700 3650
Connection ~ 1650 3550
Wire Wire Line
	1650 3750 1700 3750
Connection ~ 1650 3650
Text Label 1150 3450 0    60   ~ 0
VCC_3V30
Connection ~ 1450 3450
$Comp
L GND #PWR?
U 1 1 58D5928D
P 1450 3850
F 0 "#PWR?" H 1450 3600 50  0001 C CNN
F 1 "GND" H 1450 3700 50  0001 C CNN
F 2 "" H 1450 3850 50  0000 C CNN
F 3 "" H 1450 3850 50  0000 C CNN
	1    1450 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3800 1450 3850
Wire Wire Line
	4550 5350 5000 5350
Wire Wire Line
	4550 5450 5000 5450
Wire Wire Line
	4550 5550 5000 5550
Wire Wire Line
	4550 5650 5000 5650
Text Label 4550 5250 0    60   ~ 0
VCC_3V30
Wire Wire Line
	4550 5250 5000 5250
$Comp
L Q_PMOS_GSD Q?
U 1 1 58D5EA03
P 5550 3600
F 0 "Q?" V 5850 3600 50  0000 C CNN
F 1 "IRLML6401" V 5750 3600 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5750 3700 50  0001 C CNN
F 3 "" H 5550 3600 50  0000 C CNN
	1    5550 3600
	0    1    -1   0   
$EndComp
$Comp
L R R?
U 1 1 58D5EA04
P 5150 3700
F 0 "R?" V 5050 3700 50  0000 C CNN
F 1 "22k" V 5150 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5080 3700 50  0001 C CNN
F 3 "" H 5150 3700 50  0000 C CNN
	1    5150 3700
	-1   0    0    1   
$EndComp
$Comp
L MICROSD P?
U 1 1 58D5EA05
P 7050 4700
F 0 "P?" H 7300 4050 60  0000 C CNN
F 1 "MICROSD" V 7300 4700 60  0000 C CNN
F 2 "AMesser_Connectors:Hirose_DM3AT" H 7050 4900 60  0001 C CNN
F 3 "" H 7050 4900 60  0000 C CNN
	1    7050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 4800 6650 4800
Wire Wire Line
	6200 4600 6650 4600
Wire Wire Line
	4550 4700 6650 4700
Wire Wire Line
	4550 4500 6650 4500
Wire Wire Line
	4550 4300 6650 4300
Wire Wire Line
	4550 4200 6650 4200
Wire Wire Line
	4550 4100 6650 4100
Wire Wire Line
	6200 4600 6200 5650
Wire Wire Line
	6550 4400 6650 4400
Wire Wire Line
	6200 5100 6650 5100
Connection ~ 6200 5100
Wire Wire Line
	4550 5000 6650 5000
Wire Wire Line
	6050 4050 6050 4300
Connection ~ 6050 4300
Wire Wire Line
	6150 4050 6150 4100
Connection ~ 6150 4100
Wire Wire Line
	6250 4050 6250 4200
Connection ~ 6250 4200
Wire Wire Line
	6350 4050 6350 4700
Connection ~ 6350 4700
Wire Wire Line
	6450 4050 6450 4800
Connection ~ 6450 4800
$Comp
L R_ARRAY_4 R?
U 1 1 58D5EA14
P 6150 3900
F 0 "R?" V 6100 4100 60  0000 C CNN
F 1 "68k" V 6150 3900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6150 3900 60  0001 C CNN
F 3 "" H 6150 3900 60  0001 C CNN
	1    6150 3900
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58D5EA15
P 6450 3900
F 0 "R?" V 6400 4100 60  0000 C CNN
F 1 "68k" V 6450 3900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6450 3900 60  0001 C CNN
F 3 "" H 6450 3900 60  0001 C CNN
	1    6450 3900
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58D5EA16
P 6250 3900
F 0 "R?" V 6200 4100 60  0000 C CNN
F 1 "68k" V 6250 3900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6250 3900 60  0001 C CNN
F 3 "" H 6250 3900 60  0001 C CNN
	1    6250 3900
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58D5EA17
P 6350 3900
F 0 "R?" V 6300 4100 60  0000 C CNN
F 1 "68k" V 6350 3900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6350 3900 60  0001 C CNN
F 3 "" H 6350 3900 60  0001 C CNN
	1    6350 3900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D5EA3D
P 6750 3700
F 0 "C?" H 6775 3800 50  0000 L CNN
F 1 "4.7µF" H 6775 3600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6788 3550 50  0001 C CNN
F 3 "" H 6750 3700 50  0000 C CNN
	1    6750 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3500 6050 3750
Wire Wire Line
	6000 3500 6750 3500
Wire Wire Line
	6150 3500 6150 3750
Wire Wire Line
	6250 3500 6250 3750
Connection ~ 6150 3500
Wire Wire Line
	6350 3500 6350 3750
Connection ~ 6250 3500
Wire Wire Line
	6450 3500 6450 3750
Connection ~ 6350 3500
Connection ~ 6450 3500
$Comp
L GND #PWR?
U 1 1 58D5EA3F
P 6750 3900
F 0 "#PWR?" H 6750 3650 50  0001 C CNN
F 1 "GND" H 6750 3750 50  0001 C CNN
F 2 "" H 6750 3900 50  0000 C CNN
F 3 "" H 6750 3900 50  0000 C CNN
	1    6750 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3850 6750 3900
Wire Wire Line
	6750 3500 6750 3550
Connection ~ 6550 3500
$Comp
L R R?
U 1 1 58D5EA4C
P 5350 3900
F 0 "R?" V 5450 3900 50  0000 C CNN
F 1 "1.0k" V 5350 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5280 3900 50  0001 C CNN
F 3 "" H 5350 3900 50  0000 C CNN
	1    5350 3900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 58D5EA57
P 6050 3900
F 0 "R?" V 6100 3700 50  0000 C CNN
F 1 "10k" V 6050 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5980 3900 50  0001 C CNN
F 3 "" H 6050 3900 50  0000 C CNN
	1    6050 3900
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 58D613A9
P 6400 5450
F 0 "R?" V 6480 5450 50  0000 C CNN
F 1 "1M" V 6400 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 6330 5450 50  0001 C CNN
F 3 "" H 6400 5450 50  0000 C CNN
	1    6400 5450
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58D613AF
P 6600 5450
F 0 "C?" H 6625 5550 50  0000 L CNN
F 1 "100n" H 6625 5350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6638 5300 50  0001 C CNN
F 3 "" H 6600 5450 50  0000 C CNN
	1    6600 5450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6400 5250 6650 5250
Wire Wire Line
	6400 5250 6400 5300
Wire Wire Line
	6600 5250 6600 5300
Connection ~ 6600 5250
Wire Wire Line
	6600 5600 6600 5700
Wire Wire Line
	6200 5650 6600 5650
Wire Wire Line
	6400 5650 6400 5600
$Comp
L GND #PWR?
U 1 1 58D613BC
P 6600 5700
F 0 "#PWR?" H 6600 5450 50  0001 C CNN
F 1 "GND" H 6600 5550 50  0001 C CNN
F 2 "" H 6600 5700 50  0000 C CNN
F 3 "" H 6600 5700 50  0000 C CNN
	1    6600 5700
	-1   0    0    -1  
$EndComp
Connection ~ 6600 5650
$Comp
L L_Small L?
U 1 1 58D615A7
P 5900 3500
F 0 "L?" V 6000 3550 50  0000 R CNN
F 1 "470n" V 5950 3550 50  0000 R CNN
F 2 "" H 5900 3500 50  0000 C CNN
F 3 "" H 5900 3500 50  0000 C CNN
	1    5900 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6550 3500 6550 3500
Wire Wire Line
	6550 3500 6550 4400
Text Label 4550 3500 0    60   ~ 0
VCC_3V30
Text Label 4550 4100 0    60   ~ 0
SDCARD_DAT2
Text Label 4550 4200 0    60   ~ 0
SDCARD_DAT3
Text Label 4550 4300 0    60   ~ 0
SDCARD_CMD
Text Label 4550 4500 0    60   ~ 0
SDCARD_CLK
Text Label 4550 4700 0    60   ~ 0
SDCARD_DAT0
Text Label 4550 4800 0    60   ~ 0
SDCARD_DAT1
Connection ~ 6400 5650
Text Label 4550 5000 0    60   ~ 0
SDCARD_CD
Wire Wire Line
	5550 3800 5550 3900
Wire Wire Line
	5550 3900 5500 3900
Wire Wire Line
	5150 3550 5150 3500
Wire Wire Line
	4550 3500 5350 3500
Connection ~ 5150 3500
Wire Wire Line
	5150 3850 5150 3900
Wire Wire Line
	4550 3900 5200 3900
Connection ~ 5150 3900
Text Label 4550 3900 0    60   ~ 0
~SDCARD_PEN~
Connection ~ 6050 3500
Wire Wire Line
	5750 3500 5800 3500
Text Label 3950 4350 2    60   ~ 0
SDCARD_DAT2
Text Label 3950 4450 2    60   ~ 0
SDCARD_DAT3
Text Label 3950 4550 2    60   ~ 0
SDCARD_CMD
Text Label 3950 4650 2    60   ~ 0
SDCARD_CLK
Text Label 3950 4750 2    60   ~ 0
SDCARD_DAT0
Text Label 3950 4850 2    60   ~ 0
SDCARD_DAT1
Text Label 3950 4950 2    60   ~ 0
SDCARD_CD
Text Label 3950 4250 2    60   ~ 0
~SDCARD_PEN~
Wire Wire Line
	3300 4250 3950 4250
Wire Wire Line
	3950 4350 3300 4350
Wire Wire Line
	3300 4450 3950 4450
Wire Wire Line
	3300 4550 3950 4550
Wire Wire Line
	3300 4650 3950 4650
Wire Wire Line
	3300 4750 3950 4750
Wire Wire Line
	3300 4850 3950 4850
Wire Wire Line
	3300 4950 3950 4950
$Comp
L CONN_02X05 P?
U 1 1 58D65CD9
P 5250 5450
F 0 "P?" H 5250 5750 50  0000 C CNN
F 1 "CONN_02X05" H 5250 5150 50  0000 C CNN
F 2 "" H 5250 4250 50  0000 C CNN
F 3 "" H 5250 4250 50  0000 C CNN
	1    5250 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58D6655D
P 6000 5300
F 0 "#PWR?" H 6000 5050 50  0001 C CNN
F 1 "GND" H 6000 5150 50  0001 C CNN
F 2 "" H 6000 5300 50  0000 C CNN
F 3 "" H 6000 5300 50  0000 C CNN
	1    6000 5300
	1    0    0    -1  
$EndComp
Text Label 5900 5350 2    60   ~ 0
EXT_IO1
Text Label 4550 5450 0    60   ~ 0
EXT_IO2
Text Label 5900 5450 2    60   ~ 0
EXT_IO3
Text Label 4550 5550 0    60   ~ 0
EXT_IO4
Text Label 5900 5550 2    60   ~ 0
EXT_IO5
Text Label 4550 5650 0    60   ~ 0
EXT_IO6
Text Label 5900 5650 2    60   ~ 0
EXT_IO7
Text Label 4550 5350 0    60   ~ 0
EXT_IO0
Wire Wire Line
	5500 5350 5900 5350
Wire Wire Line
	5500 5450 5900 5450
Wire Wire Line
	5500 5550 5900 5550
Wire Wire Line
	5500 5650 5900 5650
Wire Wire Line
	5500 5250 6000 5250
Wire Wire Line
	6000 5250 6000 5300
Text Label 3950 3450 2    60   ~ 0
EXT_IO0
Text Label 3950 3550 2    60   ~ 0
EXT_IO1
Text Label 3950 3650 2    60   ~ 0
EXT_IO2
Text Label 3950 3750 2    60   ~ 0
EXT_IO3
Text Label 3950 3850 2    60   ~ 0
EXT_IO4
Text Label 3950 3950 2    60   ~ 0
EXT_IO5
Text Label 3950 4050 2    60   ~ 0
EXT_IO6
Text Label 3950 4150 2    60   ~ 0
EXT_IO7
Wire Wire Line
	3300 3450 3950 3450
Wire Wire Line
	3300 3550 3950 3550
Wire Wire Line
	3300 3650 3950 3650
Wire Wire Line
	3300 3750 3950 3750
Wire Wire Line
	3300 3850 3950 3850
Wire Wire Line
	3300 3950 3950 3950
Wire Wire Line
	3300 4050 3950 4050
Wire Wire Line
	3300 4150 3950 4150
$Comp
L C C?
U 1 1 58D6A089
P 1750 1850
F 0 "C?" H 1775 1950 50  0000 L CNN
F 1 "100n" H 1775 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1788 1700 50  0001 C CNN
F 3 "" H 1750 1850 50  0000 C CNN
	1    1750 1850
	1    0    0    -1  
$EndComp
Text GLabel 1400 5500 0    60   Input ~ 0
VCC_3V30
Text GLabel 1400 5700 0    60   Input ~ 0
VCC_1V20
Text Label 1900 5500 2    60   ~ 0
VCC_3V30
Text Label 1900 5700 2    60   ~ 0
VCC_1V20
Wire Wire Line
	1400 5500 1900 5500
Wire Wire Line
	1900 5700 1400 5700
$EndSCHEMATC
