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
Sheet 6 6
Title "Cyclone V Evaluation Board"
Date "2017-03-29"
Rev "1"
Comp "Copyright (c) 2017 Andreas Messer"
Comment1 "Cyclone V Power & Extra Port"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L C C?
U 1 1 58D4AE61
P 1100 6150
F 0 "C?" H 1125 6250 50  0000 L CNN
F 1 "4µ7" H 900 6050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1138 6000 50  0001 C CNN
F 3 "" H 1100 6150 50  0000 C CNN
	1    1100 6150
	1    0    0    -1  
$EndComp
$Comp
L 5CEFA2F23 U?
U 2 1 58D588FB
P 2150 6650
F 0 "U?" H 1650 7450 60  0000 C CNN
F 1 "5CEFA2F23" H 2150 5750 60  0000 C CNN
F 2 "" H 2150 6750 60  0001 C CNN
F 3 "" H 2150 6750 60  0001 C CNN
	2    2150 6650
	1    0    0    -1  
$EndComp
Text Label 800  5950 0    60   ~ 0
VCC_3V30
$Comp
L GND #PWR?
U 1 1 58D5928D
P 900 6400
F 0 "#PWR?" H 900 6150 50  0001 C CNN
F 1 "GND" H 900 6250 50  0001 C CNN
F 2 "" H 900 6400 50  0000 C CNN
F 3 "" H 900 6400 50  0000 C CNN
	1    900  6400
	1    0    0    -1  
$EndComp
Text Label 3850 5950 0    60   ~ 0
VCC_3V30
$Comp
L GND #PWR?
U 1 1 58D6655D
P 5350 7100
F 0 "#PWR?" H 5350 6850 50  0001 C CNN
F 1 "GND" H 5350 6950 50  0001 C CNN
F 2 "" H 5350 7100 50  0000 C CNN
F 3 "" H 5350 7100 50  0000 C CNN
	1    5350 7100
	1    0    0    -1  
$EndComp
Text Label 5250 6250 2    60   ~ 0
EXT_IO1
Text Label 3850 6350 0    60   ~ 0
EXT_IO2
Text Label 5250 6350 2    60   ~ 0
EXT_IO3
Text Label 3850 6450 0    60   ~ 0
EXT_IO4
Text Label 5250 6450 2    60   ~ 0
EXT_IO5
Text Label 3850 6550 0    60   ~ 0
EXT_IO6
Text Label 5250 6550 2    60   ~ 0
EXT_IO7
Text Label 3850 6250 0    60   ~ 0
EXT_IO0
Text Label 3600 5950 2    60   ~ 0
EXT_IO0
Text Label 3600 6050 2    60   ~ 0
EXT_IO1
Text Label 3600 6150 2    60   ~ 0
EXT_IO2
Text Label 3600 6250 2    60   ~ 0
EXT_IO3
Text Label 3600 6350 2    60   ~ 0
EXT_IO4
Text Label 3600 6450 2    60   ~ 0
EXT_IO5
Text Label 3600 6550 2    60   ~ 0
EXT_IO6
Text Label 3600 6650 2    60   ~ 0
EXT_IO7
Text GLabel 1150 750  0    60   Input ~ 0
VCC_3V30
Text GLabel 1150 950  0    60   Input ~ 0
VCC_1V10
Text Label 1650 750  2    60   ~ 0
VCC_3V30
Text Label 1650 950  2    60   ~ 0
VCC_1V10
Text Label 3600 6750 2    60   ~ 0
EXT_IO8
Text Label 3600 6850 2    60   ~ 0
EXT_IO9
Text Label 3600 6950 2    60   ~ 0
EXT_IO10
Text Label 3600 7050 2    60   ~ 0
EXT_IO11
Text Label 3600 7150 2    60   ~ 0
EXT_IO12
Text Label 3600 7250 2    60   ~ 0
EXT_IO13
Text Label 3600 7350 2    60   ~ 0
EXT_IO14
Text Label 3600 7450 2    60   ~ 0
EXT_IO15
Text Label 3850 7150 0    60   ~ 0
EXT_IO8
Text Label 5250 7150 2    60   ~ 0
EXT_IO9
Text Label 3850 7250 0    60   ~ 0
EXT_IO10
Text Label 5250 7250 2    60   ~ 0
EXT_IO11
Text Label 3850 7350 0    60   ~ 0
EXT_IO12
Text Label 5250 7350 2    60   ~ 0
EXT_IO13
Text Label 3850 7450 0    60   ~ 0
EXT_IO14
Text Label 5250 7450 2    60   ~ 0
EXT_IO15
$Comp
L CONN_02X05 P?
U 1 1 58D4FBA7
P 4550 6350
F 0 "P?" H 4550 6650 50  0000 C CNN
F 1 "CONN_02X05" H 4550 6050 50  0000 C CNN
F 2 "" H 4550 5150 50  0000 C CNN
F 3 "" H 4550 5150 50  0000 C CNN
	1    4550 6350
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 P?
U 1 1 58D4FC31
P 4550 7250
F 0 "P?" H 4550 7550 50  0000 C CNN
F 1 "CONN_02X05" H 4550 6950 50  0000 C CNN
F 2 "" H 4550 6050 50  0000 C CNN
F 3 "" H 4550 6050 50  0000 C CNN
	1    4550 7250
	1    0    0    -1  
$EndComp
$Comp
L 5CEFA2F23 U?
U 1 1 58DB14C8
P 8800 3600
F 0 "U?" H 7050 6600 60  0000 C CNN
F 1 "5CEFA2F23" H 8800 1100 60  0000 C CNN
F 2 "" H 8800 3700 60  0001 C CNN
F 3 "" H 8800 3700 60  0001 C CNN
	1    8800 3600
	1    0    0    -1  
$EndComp
NoConn ~ 9200 700 
NoConn ~ 9200 800 
NoConn ~ 9200 900 
NoConn ~ 9200 1000
$Comp
L GND #PWR?
U 1 1 58D81456
P 8450 5750
F 0 "#PWR?" H 8450 5500 50  0001 C CNN
F 1 "GND" H 8450 5600 50  0001 C CNN
F 2 "" H 8450 5750 50  0000 C CNN
F 3 "" H 8450 5750 50  0000 C CNN
	1    8450 5750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58D815E1
P 10850 5750
F 0 "#PWR?" H 10850 5500 50  0001 C CNN
F 1 "GND" H 10850 5600 50  0001 C CNN
F 2 "" H 10850 5750 50  0000 C CNN
F 3 "" H 10850 5750 50  0000 C CNN
	1    10850 5750
	1    0    0    -1  
$EndComp
Text GLabel 1150 1150 0    60   Input ~ 0
VCC_2V50
Text Label 1650 1150 2    60   ~ 0
VCC_2V50
$Comp
L C C?
U 1 1 58DBFB15
P 6600 900
F 0 "C?" H 6625 1000 50  0000 L CNN
F 1 "4µ7" H 6400 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6638 750 50  0001 C CNN
F 3 "" H 6600 900 50  0000 C CNN
	1    6600 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFCDD
P 6400 900
F 0 "C?" H 6425 1000 50  0000 L CNN
F 1 "4µ7" H 6200 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6438 750 50  0001 C CNN
F 3 "" H 6400 900 50  0000 C CNN
	1    6400 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFD44
P 6200 900
F 0 "C?" H 6225 1000 50  0000 L CNN
F 1 "4µ7" H 6000 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6238 750 50  0001 C CNN
F 3 "" H 6200 900 50  0000 C CNN
	1    6200 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFD4A
P 6000 900
F 0 "C?" H 6025 1000 50  0000 L CNN
F 1 "4µ7" H 5800 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6038 750 50  0001 C CNN
F 3 "" H 6000 900 50  0000 C CNN
	1    6000 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFE24
P 5800 900
F 0 "C?" H 5825 1000 50  0000 L CNN
F 1 "4µ7" H 5600 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5838 750 50  0001 C CNN
F 3 "" H 5800 900 50  0000 C CNN
	1    5800 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFE2A
P 5600 900
F 0 "C?" H 5625 1000 50  0000 L CNN
F 1 "4µ7" H 5400 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5638 750 50  0001 C CNN
F 3 "" H 5600 900 50  0000 C CNN
	1    5600 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFE30
P 5400 900
F 0 "C?" H 5425 1000 50  0000 L CNN
F 1 "4µ7" H 5200 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5438 750 50  0001 C CNN
F 3 "" H 5400 900 50  0000 C CNN
	1    5400 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DBFE36
P 5200 900
F 0 "C?" H 5225 1000 50  0000 L CNN
F 1 "4µ7" H 5000 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5238 750 50  0001 C CNN
F 3 "" H 5200 900 50  0000 C CNN
	1    5200 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC0051
P 5000 900
F 0 "C?" H 5025 1000 50  0000 L CNN
F 1 "4µ7" H 4800 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5038 750 50  0001 C CNN
F 3 "" H 5000 900 50  0000 C CNN
	1    5000 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC0057
P 4800 900
F 0 "C?" H 4825 1000 50  0000 L CNN
F 1 "4µ7" H 4600 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4838 750 50  0001 C CNN
F 3 "" H 4800 900 50  0000 C CNN
	1    4800 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC005D
P 4600 900
F 0 "C?" H 4625 1000 50  0000 L CNN
F 1 "4µ7" H 4400 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4638 750 50  0001 C CNN
F 3 "" H 4600 900 50  0000 C CNN
	1    4600 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC0063
P 4400 900
F 0 "C?" H 4425 1000 50  0000 L CNN
F 1 "4µ7" H 4200 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4438 750 50  0001 C CNN
F 3 "" H 4400 900 50  0000 C CNN
	1    4400 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC0069
P 4200 900
F 0 "C?" H 4225 1000 50  0000 L CNN
F 1 "4µ7" H 4000 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4238 750 50  0001 C CNN
F 3 "" H 4200 900 50  0000 C CNN
	1    4200 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC006F
P 4000 900
F 0 "C?" H 4025 1000 50  0000 L CNN
F 1 "4µ7" H 3800 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4038 750 50  0001 C CNN
F 3 "" H 4000 900 50  0000 C CNN
	1    4000 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC0075
P 3800 900
F 0 "C?" H 3825 1000 50  0000 L CNN
F 1 "4µ7" H 3600 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3838 750 50  0001 C CNN
F 3 "" H 3800 900 50  0000 C CNN
	1    3800 900 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC007B
P 3600 900
F 0 "C?" H 3625 1000 50  0000 L CNN
F 1 "4µ7" H 3400 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3638 750 50  0001 C CNN
F 3 "" H 3600 900 50  0000 C CNN
	1    3600 900 
	1    0    0    -1  
$EndComp
Text Label 3450 700  0    60   ~ 0
VCC_1V10
$Comp
L GND #PWR?
U 1 1 58DC32E7
P 3600 1150
F 0 "#PWR?" H 3600 900 50  0001 C CNN
F 1 "GND" H 3600 1000 50  0001 C CNN
F 2 "" H 3600 1150 50  0000 C CNN
F 3 "" H 3600 1150 50  0000 C CNN
	1    3600 1150
	1    0    0    -1  
$EndComp
Text Label 5650 3700 0    60   ~ 0
VCC_2V50
$Comp
L C C?
U 1 1 58DC5A55
P 6600 5200
F 0 "C?" H 6625 5300 50  0000 L CNN
F 1 "100n" H 6400 5100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6638 5050 50  0001 C CNN
F 3 "" H 6600 5200 50  0000 C CNN
	1    6600 5200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC5C14
P 6400 5200
F 0 "C?" H 6425 5300 50  0000 L CNN
F 1 "100n" H 6200 5100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6438 5050 50  0001 C CNN
F 3 "" H 6400 5200 50  0000 C CNN
	1    6400 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DC5E95
P 6400 5450
F 0 "#PWR?" H 6400 5200 50  0001 C CNN
F 1 "GND" H 6400 5300 50  0001 C CNN
F 2 "" H 6400 5450 50  0000 C CNN
F 3 "" H 6400 5450 50  0000 C CNN
	1    6400 5450
	1    0    0    -1  
$EndComp
$Comp
L L_Small L?
U 1 1 58DC6483
P 6250 5000
F 0 "L?" H 6280 5040 50  0000 L CNN
F 1 "50Ohm@100MHz" V 6300 4700 50  0000 L CNN
F 2 "" H 6250 5000 50  0000 C CNN
F 3 "" H 6250 5000 50  0000 C CNN
	1    6250 5000
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 58DC73F7
P 6600 3900
F 0 "C?" H 6625 4000 50  0000 L CNN
F 1 "4µ7" H 6400 3800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6638 3750 50  0001 C CNN
F 3 "" H 6600 3900 50  0000 C CNN
	1    6600 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DC7518
P 6600 4100
F 0 "#PWR?" H 6600 3850 50  0001 C CNN
F 1 "GND" H 6600 3950 50  0001 C CNN
F 2 "" H 6600 4100 50  0000 C CNN
F 3 "" H 6600 4100 50  0000 C CNN
	1    6600 4100
	1    0    0    -1  
$EndComp
Text Label 5650 4200 0    60   ~ 0
VCCPD_B5A
Text Label 5650 4300 0    60   ~ 0
VCCPD_B5B
$Comp
L C C?
U 1 1 58DC8A4B
P 6550 4500
F 0 "C?" H 6575 4600 50  0000 L CNN
F 1 "4µ7" H 6350 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6588 4350 50  0001 C CNN
F 3 "" H 6550 4500 50  0000 C CNN
	1    6550 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DC8AF7
P 6350 4500
F 0 "C?" H 6375 4600 50  0000 L CNN
F 1 "4µ7" H 6150 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6388 4350 50  0001 C CNN
F 3 "" H 6350 4500 50  0000 C CNN
	1    6350 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DC9195
P 6350 4750
F 0 "#PWR?" H 6350 4500 50  0001 C CNN
F 1 "GND" H 6350 4600 50  0001 C CNN
F 2 "" H 6350 4750 50  0000 C CNN
F 3 "" H 6350 4750 50  0000 C CNN
	1    6350 4750
	1    0    0    -1  
$EndComp
Text Label 5650 4900 0    60   ~ 0
VCC_2V50
$Comp
L C C?
U 1 1 58DCA117
P 5950 5100
F 0 "C?" H 5975 5200 50  0000 L CNN
F 1 "4µ7" H 5750 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5988 4950 50  0001 C CNN
F 3 "" H 5950 5100 50  0000 C CNN
	1    5950 5100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DCA1B7
P 5750 5100
F 0 "C?" H 5775 5200 50  0000 L CNN
F 1 "4µ7" H 5550 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5788 4950 50  0001 C CNN
F 3 "" H 5750 5100 50  0000 C CNN
	1    5750 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DCA226
P 5750 5350
F 0 "#PWR?" H 5750 5100 50  0001 C CNN
F 1 "GND" H 5750 5200 50  0001 C CNN
F 2 "" H 5750 5350 50  0000 C CNN
F 3 "" H 5750 5350 50  0000 C CNN
	1    5750 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DCA82B
P 6600 6000
F 0 "#PWR?" H 6600 5750 50  0001 C CNN
F 1 "GND" H 6600 5850 50  0001 C CNN
F 2 "" H 6600 6000 50  0000 C CNN
F 3 "" H 6600 6000 50  0000 C CNN
	1    6600 6000
	1    0    0    -1  
$EndComp
Text Label 5650 5600 0    60   ~ 0
VCC_2V50
$Comp
L C C?
U 1 1 58DCB8F2
P 6600 5800
F 0 "C?" H 6625 5900 50  0000 L CNN
F 1 "4µ7" H 6400 5700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6638 5650 50  0001 C CNN
F 3 "" H 6600 5800 50  0000 C CNN
	1    6600 5800
	1    0    0    -1  
$EndComp
Text Label 5650 3300 0    60   ~ 0
VCC_2V50
Text Label 1650 1500 2    60   ~ 0
VCCPD_B5A
Text GLabel 1100 1500 0    60   Input ~ 0
VCCPD_B5A
Text GLabel 1100 1650 0    60   Input ~ 0
VCCPD_B5B
Text Label 1650 1650 2    60   ~ 0
VCCPD_B5B
Text Label 5650 3400 0    60   ~ 0
VCC_3V30
$Comp
L R R?
U 1 1 58DCF6B0
P 6750 6400
F 0 "R?" V 6830 6400 50  0000 C CNN
F 1 "2k" V 6750 6400 50  0000 C CNN
F 2 "" V 6680 6400 50  0000 C CNN
F 3 "" H 6750 6400 50  0000 C CNN
	1    6750 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58DCFA56
P 6750 6600
F 0 "#PWR?" H 6750 6350 50  0001 C CNN
F 1 "GND" H 6750 6450 50  0001 C CNN
F 2 "" H 6750 6600 50  0000 C CNN
F 3 "" H 6750 6600 50  0000 C CNN
	1    6750 6600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DD33D9
P 900 6150
F 0 "C?" H 925 6250 50  0000 L CNN
F 1 "4µ7" H 700 6050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 938 6000 50  0001 C CNN
F 3 "" H 900 6150 50  0000 C CNN
	1    900  6150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DD41B2
P 4550 5950
F 0 "C?" H 4575 6050 50  0000 L CNN
F 1 "4µ7" H 4350 5850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4588 5800 50  0001 C CNN
F 3 "" H 4550 5950 50  0000 C CNN
	1    4550 5950
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 58DD4652
P 4550 6850
F 0 "C?" H 4575 6950 50  0000 L CNN
F 1 "4µ7" H 4350 6750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4588 6700 50  0001 C CNN
F 3 "" H 4550 6850 50  0000 C CNN
	1    4550 6850
	0    1    1    0   
$EndComp
Wire Wire Line
	800  5950 1350 5950
Wire Wire Line
	1100 5950 1100 6000
Wire Wire Line
	1300 5950 1300 6250
Wire Wire Line
	1300 6050 1350 6050
Connection ~ 1300 5950
Wire Wire Line
	1300 6150 1350 6150
Connection ~ 1300 6050
Wire Wire Line
	1300 6250 1350 6250
Connection ~ 1300 6150
Connection ~ 1100 5950
Wire Wire Line
	1100 6300 1100 6350
Wire Wire Line
	3850 6250 4300 6250
Wire Wire Line
	3850 6350 4300 6350
Wire Wire Line
	3850 6450 4300 6450
Wire Wire Line
	3850 6550 4300 6550
Wire Wire Line
	4250 5950 4250 7050
Wire Wire Line
	4250 6150 4300 6150
Wire Wire Line
	4800 6250 5250 6250
Wire Wire Line
	4800 6350 5250 6350
Wire Wire Line
	4800 6450 5250 6450
Wire Wire Line
	4800 6550 5250 6550
Wire Wire Line
	4800 6150 5350 6150
Wire Wire Line
	5350 6150 5350 7100
Wire Wire Line
	2950 5950 3600 5950
Wire Wire Line
	2950 6050 3600 6050
Wire Wire Line
	2950 6150 3600 6150
Wire Wire Line
	2950 6250 3600 6250
Wire Wire Line
	2950 6350 3600 6350
Wire Wire Line
	2950 6450 3600 6450
Wire Wire Line
	2950 6550 3600 6550
Wire Wire Line
	2950 6650 3600 6650
Wire Wire Line
	1150 750  1650 750 
Wire Wire Line
	2950 6750 3600 6750
Wire Wire Line
	2950 6850 3600 6850
Wire Wire Line
	2950 6950 3600 6950
Wire Wire Line
	2950 7050 3600 7050
Wire Wire Line
	2950 7150 3600 7150
Wire Wire Line
	2950 7250 3600 7250
Wire Wire Line
	2950 7350 3600 7350
Wire Wire Line
	2950 7450 3600 7450
Wire Wire Line
	4800 7150 5250 7150
Wire Wire Line
	4800 7250 5250 7250
Wire Wire Line
	4800 7350 5250 7350
Wire Wire Line
	4800 7450 5250 7450
Wire Wire Line
	4300 7150 3850 7150
Wire Wire Line
	4300 7250 3850 7250
Wire Wire Line
	4300 7350 3850 7350
Wire Wire Line
	4300 7450 3850 7450
Wire Wire Line
	4800 7050 5350 7050
Connection ~ 5350 7050
Wire Wire Line
	4250 7050 4300 7050
Connection ~ 4250 6150
Wire Wire Line
	1150 950  1650 950 
Wire Wire Line
	3450 700  6800 700 
Wire Wire Line
	6750 700  6750 3200
Wire Wire Line
	6750 800  6800 800 
Wire Wire Line
	6750 900  6800 900 
Connection ~ 6750 800 
Wire Wire Line
	6750 1000 6800 1000
Connection ~ 6750 900 
Wire Wire Line
	6750 1100 6800 1100
Connection ~ 6750 1000
Wire Wire Line
	6750 1200 6800 1200
Connection ~ 6750 1100
Wire Wire Line
	6750 1300 6800 1300
Connection ~ 6750 1200
Wire Wire Line
	6750 1400 6800 1400
Connection ~ 6750 1300
Wire Wire Line
	6750 1500 6800 1500
Connection ~ 6750 1400
Wire Wire Line
	6750 1600 6800 1600
Connection ~ 6750 1500
Wire Wire Line
	6750 1700 6800 1700
Connection ~ 6750 1600
Wire Wire Line
	6750 1800 6800 1800
Connection ~ 6750 1700
Wire Wire Line
	6750 1900 6800 1900
Connection ~ 6750 1800
Wire Wire Line
	6750 2000 6800 2000
Connection ~ 6750 1900
Wire Wire Line
	6750 2100 6800 2100
Connection ~ 6750 2000
Wire Wire Line
	6750 2200 6800 2200
Connection ~ 6750 2100
Wire Wire Line
	6750 2300 6800 2300
Connection ~ 6750 2200
Wire Wire Line
	6750 2400 6800 2400
Connection ~ 6750 2300
Wire Wire Line
	6750 2500 6800 2500
Connection ~ 6750 2400
Wire Wire Line
	6750 2600 6800 2600
Connection ~ 6750 2500
Wire Wire Line
	6750 2700 6800 2700
Connection ~ 6750 2600
Wire Wire Line
	6750 2800 6800 2800
Connection ~ 6750 2700
Wire Wire Line
	6750 2900 6800 2900
Connection ~ 6750 2800
Wire Wire Line
	6750 3000 6800 3000
Connection ~ 6750 2900
Wire Wire Line
	6750 3100 6800 3100
Connection ~ 6750 3000
Wire Wire Line
	6750 3200 6800 3200
Connection ~ 6750 3100
Wire Wire Line
	6350 5000 6800 5000
Wire Wire Line
	6750 5000 6750 5500
Wire Wire Line
	6750 5100 6800 5100
Wire Wire Line
	6750 5200 6800 5200
Connection ~ 6750 5100
Wire Wire Line
	6750 5300 6800 5300
Connection ~ 6750 5200
Wire Wire Line
	6750 5400 6800 5400
Connection ~ 6750 5300
Wire Wire Line
	6750 5500 6800 5500
Connection ~ 6750 5400
Wire Wire Line
	5650 5600 6800 5600
Wire Wire Line
	6750 5600 6750 6100
Wire Wire Line
	6750 5700 6800 5700
Wire Wire Line
	6750 5800 6800 5800
Connection ~ 6750 5700
Wire Wire Line
	6750 5900 6800 5900
Connection ~ 6750 5800
Wire Wire Line
	6750 6000 6800 6000
Connection ~ 6750 5900
Wire Wire Line
	6750 6100 6800 6100
Connection ~ 6750 6000
Wire Wire Line
	8400 700  8450 700 
Wire Wire Line
	8450 700  8450 5750
Wire Wire Line
	8450 800  8400 800 
Wire Wire Line
	8450 900  8400 900 
Connection ~ 8450 800 
Wire Wire Line
	8450 1000 8400 1000
Connection ~ 8450 900 
Wire Wire Line
	8450 1100 8400 1100
Connection ~ 8450 1000
Wire Wire Line
	8400 1200 8450 1200
Wire Wire Line
	8400 1300 8450 1300
Wire Wire Line
	8400 1400 8450 1400
Wire Wire Line
	8400 1500 8450 1500
Wire Wire Line
	8400 1600 8450 1600
Wire Wire Line
	8400 1700 8450 1700
Wire Wire Line
	8400 1800 8450 1800
Wire Wire Line
	8400 1900 8450 1900
Wire Wire Line
	8400 2000 8450 2000
Wire Wire Line
	8400 2100 8450 2100
Wire Wire Line
	8400 2200 8450 2200
Wire Wire Line
	8400 2300 8450 2300
Wire Wire Line
	8400 2400 8450 2400
Wire Wire Line
	8400 2500 8450 2500
Wire Wire Line
	8400 2600 8450 2600
Wire Wire Line
	8400 2700 8450 2700
Wire Wire Line
	8400 2800 8450 2800
Wire Wire Line
	8400 2900 8450 2900
Wire Wire Line
	8400 3000 8450 3000
Wire Wire Line
	8400 3100 8450 3100
Wire Wire Line
	8400 3200 8450 3200
Wire Wire Line
	8400 3300 8450 3300
Wire Wire Line
	8400 3400 8450 3400
Wire Wire Line
	8400 3500 8450 3500
Wire Wire Line
	8400 3600 8450 3600
Wire Wire Line
	8400 3700 8450 3700
Wire Wire Line
	8400 3800 8450 3800
Wire Wire Line
	8400 3900 8450 3900
Wire Wire Line
	8400 4000 8450 4000
Wire Wire Line
	8400 4100 8450 4100
Wire Wire Line
	8400 4200 8450 4200
Wire Wire Line
	8400 4300 8450 4300
Wire Wire Line
	8400 4400 8450 4400
Wire Wire Line
	8400 4500 8450 4500
Wire Wire Line
	8400 4600 8450 4600
Wire Wire Line
	8400 4700 8450 4700
Wire Wire Line
	8400 4800 8450 4800
Wire Wire Line
	8400 4900 8450 4900
Wire Wire Line
	8400 5000 8450 5000
Wire Wire Line
	8400 5100 8450 5100
Wire Wire Line
	8400 5200 8450 5200
Wire Wire Line
	8400 5300 8450 5300
Wire Wire Line
	8400 5400 8450 5400
Wire Wire Line
	8400 5500 8450 5500
Wire Wire Line
	8400 5600 8450 5600
Wire Wire Line
	8450 5700 8400 5700
Connection ~ 8450 1100
Connection ~ 8450 1200
Connection ~ 8450 1300
Connection ~ 8450 1400
Connection ~ 8450 1500
Connection ~ 8450 1600
Connection ~ 8450 1700
Connection ~ 8450 1800
Connection ~ 8450 1900
Connection ~ 8450 2000
Connection ~ 8450 2100
Connection ~ 8450 2200
Connection ~ 8450 2300
Connection ~ 8450 2400
Connection ~ 8450 2500
Connection ~ 8450 2600
Connection ~ 8450 2700
Connection ~ 8450 2800
Connection ~ 8450 2900
Connection ~ 8450 3000
Connection ~ 8450 3100
Connection ~ 8450 3200
Connection ~ 8450 3300
Connection ~ 8450 3400
Connection ~ 8450 3500
Connection ~ 8450 3600
Connection ~ 8450 3700
Connection ~ 8450 3800
Connection ~ 8450 3900
Connection ~ 8450 4000
Connection ~ 8450 4100
Connection ~ 8450 4200
Connection ~ 8450 4300
Connection ~ 8450 4400
Connection ~ 8450 4500
Connection ~ 8450 4600
Connection ~ 8450 4700
Connection ~ 8450 4800
Connection ~ 8450 4900
Connection ~ 8450 5000
Connection ~ 8450 5100
Connection ~ 8450 5200
Connection ~ 8450 5300
Connection ~ 8450 5400
Connection ~ 8450 5500
Connection ~ 8450 5600
Wire Wire Line
	10800 700  10850 700 
Wire Wire Line
	10800 800  10850 800 
Wire Wire Line
	10850 900  10800 900 
Wire Wire Line
	10850 1000 10800 1000
Wire Wire Line
	10850 1100 10800 1100
Wire Wire Line
	10850 1200 10800 1200
Wire Wire Line
	10850 1300 10800 1300
Wire Wire Line
	10850 1400 10800 1400
Wire Wire Line
	10850 1500 10800 1500
Wire Wire Line
	10850 1600 10800 1600
Wire Wire Line
	10850 1700 10800 1700
Wire Wire Line
	10850 1800 10800 1800
Wire Wire Line
	10850 1900 10800 1900
Wire Wire Line
	10850 2000 10800 2000
Wire Wire Line
	10850 2100 10800 2100
Wire Wire Line
	10850 2200 10800 2200
Wire Wire Line
	10850 2300 10800 2300
Wire Wire Line
	10850 2400 10800 2400
Wire Wire Line
	10850 2500 10800 2500
Wire Wire Line
	10850 2600 10800 2600
Wire Wire Line
	10850 2700 10800 2700
Wire Wire Line
	10850 2800 10800 2800
Wire Wire Line
	10850 2900 10800 2900
Wire Wire Line
	10850 3000 10800 3000
Wire Wire Line
	10850 3100 10800 3100
Wire Wire Line
	10850 3200 10800 3200
Wire Wire Line
	10850 3300 10800 3300
Wire Wire Line
	10850 3400 10800 3400
Wire Wire Line
	10850 3500 10800 3500
Wire Wire Line
	10850 3600 10800 3600
Wire Wire Line
	10850 3700 10800 3700
Wire Wire Line
	10850 3800 10800 3800
Wire Wire Line
	10850 3900 10800 3900
Wire Wire Line
	10850 4000 10800 4000
Wire Wire Line
	10850 4100 10800 4100
Wire Wire Line
	10850 4200 10800 4200
Wire Wire Line
	10850 4300 10800 4300
Wire Wire Line
	10850 4400 10800 4400
Wire Wire Line
	10850 4500 10800 4500
Wire Wire Line
	10850 4600 10800 4600
Wire Wire Line
	10850 4700 10800 4700
Wire Wire Line
	10850 4800 10800 4800
Wire Wire Line
	10850 4900 10800 4900
Wire Wire Line
	10850 5000 10800 5000
Wire Wire Line
	10850 5100 10800 5100
Wire Wire Line
	10850 5200 10800 5200
Wire Wire Line
	10850 5300 10800 5300
Wire Wire Line
	10850 5400 10800 5400
Wire Wire Line
	10850 5500 10800 5500
Wire Wire Line
	10850 5600 10800 5600
Wire Wire Line
	10850 700  10850 5750
Connection ~ 10850 800 
Connection ~ 10850 900 
Connection ~ 10850 1000
Connection ~ 10850 1100
Connection ~ 10850 1200
Connection ~ 10850 1300
Connection ~ 10850 1400
Connection ~ 10850 1500
Connection ~ 10850 1600
Connection ~ 10850 1700
Connection ~ 10850 1800
Connection ~ 10850 1900
Connection ~ 10850 2000
Connection ~ 10850 2100
Connection ~ 10850 2200
Connection ~ 10850 2300
Connection ~ 10850 2400
Connection ~ 10850 2500
Connection ~ 10850 2600
Connection ~ 10850 2700
Connection ~ 10850 2800
Connection ~ 10850 2900
Connection ~ 10850 3000
Connection ~ 10850 3100
Connection ~ 10850 3200
Connection ~ 10850 3300
Connection ~ 10850 3400
Connection ~ 10850 3500
Connection ~ 10850 3600
Connection ~ 10850 3700
Connection ~ 10850 3800
Connection ~ 10850 3900
Connection ~ 10850 4000
Connection ~ 10850 4100
Connection ~ 10850 4200
Connection ~ 10850 4300
Connection ~ 10850 4400
Connection ~ 10850 4500
Connection ~ 10850 4600
Connection ~ 10850 4700
Connection ~ 10850 4800
Connection ~ 10850 4900
Connection ~ 10850 5000
Connection ~ 10850 5100
Connection ~ 10850 5200
Connection ~ 10850 5300
Connection ~ 10850 5400
Connection ~ 10850 5500
Connection ~ 8450 5700
Connection ~ 10850 5600
Wire Wire Line
	1150 1150 1650 1150
Wire Wire Line
	6600 700  6600 750 
Connection ~ 6750 700 
Wire Wire Line
	6400 700  6400 750 
Connection ~ 6600 700 
Wire Wire Line
	6200 700  6200 750 
Connection ~ 6400 700 
Wire Wire Line
	6000 700  6000 750 
Connection ~ 6200 700 
Wire Wire Line
	5800 700  5800 750 
Connection ~ 6000 700 
Wire Wire Line
	5600 700  5600 750 
Connection ~ 5800 700 
Wire Wire Line
	5400 700  5400 750 
Connection ~ 5600 700 
Wire Wire Line
	5200 700  5200 750 
Connection ~ 5400 700 
Wire Wire Line
	6600 1100 6600 1050
Wire Wire Line
	3600 1100 6600 1100
Wire Wire Line
	6400 1100 6400 1050
Wire Wire Line
	6200 1100 6200 1050
Connection ~ 6400 1100
Wire Wire Line
	6000 1100 6000 1050
Connection ~ 6200 1100
Wire Wire Line
	5800 1100 5800 1050
Connection ~ 6000 1100
Wire Wire Line
	5600 1100 5600 1050
Connection ~ 5800 1100
Wire Wire Line
	5400 1100 5400 1050
Connection ~ 5600 1100
Wire Wire Line
	5200 1100 5200 1050
Connection ~ 5400 1100
Wire Wire Line
	5000 700  5000 750 
Connection ~ 5200 700 
Wire Wire Line
	4800 700  4800 750 
Connection ~ 5000 700 
Wire Wire Line
	4600 700  4600 750 
Connection ~ 4800 700 
Wire Wire Line
	4400 700  4400 750 
Connection ~ 4600 700 
Wire Wire Line
	4200 700  4200 750 
Connection ~ 4400 700 
Wire Wire Line
	4000 700  4000 750 
Connection ~ 4200 700 
Wire Wire Line
	3800 700  3800 750 
Connection ~ 4000 700 
Wire Wire Line
	3600 700  3600 750 
Connection ~ 3800 700 
Wire Wire Line
	5000 1100 5000 1050
Connection ~ 5200 1100
Wire Wire Line
	4800 1100 4800 1050
Connection ~ 5000 1100
Wire Wire Line
	4600 1100 4600 1050
Connection ~ 4800 1100
Wire Wire Line
	4400 1100 4400 1050
Connection ~ 4600 1100
Wire Wire Line
	4200 1100 4200 1050
Connection ~ 4400 1100
Wire Wire Line
	4000 1100 4000 1050
Connection ~ 4200 1100
Wire Wire Line
	3800 1100 3800 1050
Connection ~ 4000 1100
Wire Wire Line
	3600 1050 3600 1150
Connection ~ 3800 1100
Connection ~ 3600 700 
Connection ~ 3600 1100
Wire Wire Line
	5650 3400 6800 3400
Wire Wire Line
	6750 3500 6800 3500
Wire Wire Line
	6750 3600 6800 3600
Connection ~ 6750 3500
Wire Wire Line
	5650 3700 6800 3700
Wire Wire Line
	6750 3800 6800 3800
Connection ~ 6750 3700
Wire Wire Line
	6750 3900 6800 3900
Connection ~ 6750 3800
Wire Wire Line
	6750 4000 6800 4000
Connection ~ 6750 3900
Wire Wire Line
	6750 4100 6800 4100
Connection ~ 6750 4000
Connection ~ 6750 3400
Wire Wire Line
	6600 5350 6600 5400
Wire Wire Line
	6600 5400 6400 5400
Wire Wire Line
	6400 5350 6400 5450
Connection ~ 6400 5400
Wire Wire Line
	6600 5000 6600 5050
Connection ~ 6750 5000
Wire Wire Line
	6400 5000 6400 5050
Connection ~ 6600 5000
Wire Wire Line
	5650 4900 6800 4900
Wire Wire Line
	6750 4500 6750 4900
Wire Wire Line
	6750 4800 6800 4800
Wire Wire Line
	6750 4700 6800 4700
Connection ~ 6750 4800
Wire Wire Line
	6750 4600 6800 4600
Connection ~ 6750 4700
Wire Wire Line
	6750 4500 6800 4500
Connection ~ 6750 4600
Connection ~ 6400 5000
Wire Wire Line
	6600 4050 6600 4100
Wire Wire Line
	5650 4200 6800 4200
Wire Wire Line
	5650 4300 6800 4300
Wire Wire Line
	6750 4300 6750 4400
Wire Wire Line
	6750 4400 6800 4400
Connection ~ 6750 4300
Wire Wire Line
	6350 4350 6350 4200
Connection ~ 6350 4200
Wire Wire Line
	6550 4350 6550 4300
Connection ~ 6550 4300
Wire Wire Line
	6550 4650 6550 4700
Wire Wire Line
	6550 4700 6350 4700
Wire Wire Line
	6350 4650 6350 4750
Connection ~ 6350 4700
Connection ~ 6750 4900
Wire Wire Line
	6100 4900 6100 5000
Wire Wire Line
	6100 5000 6150 5000
Connection ~ 6100 4900
Wire Wire Line
	5750 5250 5750 5350
Wire Wire Line
	5750 4950 5750 4900
Connection ~ 5750 4900
Wire Wire Line
	5950 4950 5950 4900
Connection ~ 5950 4900
Wire Wire Line
	5750 5300 5950 5300
Wire Wire Line
	5950 5300 5950 5250
Connection ~ 5750 5300
Connection ~ 6750 5600
Wire Wire Line
	6600 5950 6600 6000
Wire Wire Line
	6600 5650 6600 5600
Connection ~ 6600 5600
Wire Wire Line
	5650 3300 6800 3300
Wire Wire Line
	1100 1500 1650 1500
Wire Wire Line
	1100 1650 1650 1650
Wire Wire Line
	6750 3400 6750 3600
Wire Wire Line
	6750 3700 6750 4100
Wire Wire Line
	6600 3750 6600 3700
Connection ~ 6600 3700
Wire Wire Line
	6800 6200 6750 6200
Wire Wire Line
	6750 6200 6750 6250
Wire Wire Line
	6750 6550 6750 6600
Wire Wire Line
	900  6300 900  6400
Wire Wire Line
	1100 6350 900  6350
Connection ~ 900  6350
Wire Wire Line
	900  6000 900  5950
Connection ~ 900  5950
Wire Wire Line
	3850 5950 4250 5950
Wire Wire Line
	4700 5950 4850 5950
Wire Wire Line
	4850 5950 4850 6150
Connection ~ 4850 6150
Wire Wire Line
	4400 5950 4250 5950
Connection ~ 4250 5950
Wire Wire Line
	4700 6850 4850 6850
Wire Wire Line
	4850 6850 4850 7050
Connection ~ 4850 7050
Wire Wire Line
	4400 6850 4250 6850
Connection ~ 4250 6850
$EndSCHEMATC
