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
Text Label 800  5950 0    60   ~ 0
VCC_3V30
Connection ~ 1100 5950
$Comp
L GND #PWR?
U 1 1 58D5928D
P 1100 6350
F 0 "#PWR?" H 1100 6100 50  0001 C CNN
F 1 "GND" H 1100 6200 50  0001 C CNN
F 2 "" H 1100 6350 50  0000 C CNN
F 3 "" H 1100 6350 50  0000 C CNN
	1    1100 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 6300 1100 6350
Wire Wire Line
	4050 6000 4500 6000
Wire Wire Line
	4050 6100 4500 6100
Wire Wire Line
	4050 6200 4500 6200
Wire Wire Line
	4050 6300 4500 6300
Text Label 4050 5900 0    60   ~ 0
VCC_3V30
Wire Wire Line
	4050 5900 4500 5900
$Comp
L GND #PWR?
U 1 1 58D6655D
P 5550 6550
F 0 "#PWR?" H 5550 6300 50  0001 C CNN
F 1 "GND" H 5550 6400 50  0001 C CNN
F 2 "" H 5550 6550 50  0000 C CNN
F 3 "" H 5550 6550 50  0000 C CNN
	1    5550 6550
	1    0    0    -1  
$EndComp
Text Label 5450 6000 2    60   ~ 0
EXT_IO1
Text Label 4050 6100 0    60   ~ 0
EXT_IO2
Text Label 5450 6100 2    60   ~ 0
EXT_IO3
Text Label 4050 6200 0    60   ~ 0
EXT_IO4
Text Label 5450 6200 2    60   ~ 0
EXT_IO5
Text Label 4050 6300 0    60   ~ 0
EXT_IO6
Text Label 5450 6300 2    60   ~ 0
EXT_IO7
Text Label 4050 6000 0    60   ~ 0
EXT_IO0
Wire Wire Line
	5000 6000 5450 6000
Wire Wire Line
	5000 6100 5450 6100
Wire Wire Line
	5000 6200 5450 6200
Wire Wire Line
	5000 6300 5450 6300
Wire Wire Line
	5000 5900 5550 5900
Wire Wire Line
	5550 5900 5550 6550
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
Text GLabel 1350 750  0    60   Input ~ 0
VCC_3V30
Text GLabel 1350 950  0    60   Input ~ 0
VCC_1V10
Text Label 1850 750  2    60   ~ 0
VCC_3V30
Text Label 1850 950  2    60   ~ 0
VCC_1V10
Wire Wire Line
	1350 750  1850 750 
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
Text Label 4050 6600 0    60   ~ 0
EXT_IO8
Text Label 5450 6600 2    60   ~ 0
EXT_IO9
Text Label 4050 6700 0    60   ~ 0
EXT_IO10
Text Label 5450 6700 2    60   ~ 0
EXT_IO11
Text Label 4050 6800 0    60   ~ 0
EXT_IO12
Text Label 5450 6800 2    60   ~ 0
EXT_IO13
Text Label 4050 6900 0    60   ~ 0
EXT_IO14
Text Label 5450 6900 2    60   ~ 0
EXT_IO15
Wire Wire Line
	5000 6600 5450 6600
Wire Wire Line
	5000 6700 5450 6700
Wire Wire Line
	5000 6800 5450 6800
Wire Wire Line
	5000 6900 5450 6900
Wire Wire Line
	4500 6600 4050 6600
Wire Wire Line
	4500 6700 4050 6700
Wire Wire Line
	4500 6800 4050 6800
Wire Wire Line
	4500 6900 4050 6900
Wire Wire Line
	5000 6500 5550 6500
Connection ~ 5550 6500
Wire Wire Line
	4450 6500 4500 6500
Connection ~ 4450 5900
$Comp
L CONN_02X05 P?
U 1 1 58D4FBA7
P 4750 6100
F 0 "P?" H 4750 6400 50  0000 C CNN
F 1 "CONN_02X05" H 4750 5800 50  0000 C CNN
F 2 "" H 4750 4900 50  0000 C CNN
F 3 "" H 4750 4900 50  0000 C CNN
	1    4750 6100
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 P?
U 1 1 58D4FC31
P 4750 6700
F 0 "P?" H 4750 7000 50  0000 C CNN
F 1 "CONN_02X05" H 4750 6400 50  0000 C CNN
F 2 "" H 4750 5500 50  0000 C CNN
F 3 "" H 4750 5500 50  0000 C CNN
	1    4750 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 5900 4450 6500
$Comp
L 5CEFA2F23 U?
U 1 1 58DB14C8
P 9000 3600
F 0 "U?" H 7250 6600 60  0000 C CNN
F 1 "5CEFA2F23" H 9000 1100 60  0000 C CNN
F 2 "" H 9000 3700 60  0001 C CNN
F 3 "" H 9000 3700 60  0001 C CNN
	1    9000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 950  1850 950 
Wire Wire Line
	7000 700  6950 700 
Wire Wire Line
	6950 700  6950 3200
Wire Wire Line
	6950 800  7000 800 
Wire Wire Line
	6950 900  7000 900 
Connection ~ 6950 800 
Wire Wire Line
	6950 1000 7000 1000
Connection ~ 6950 900 
Wire Wire Line
	6950 1100 7000 1100
Connection ~ 6950 1000
Wire Wire Line
	6950 1200 7000 1200
Connection ~ 6950 1100
Wire Wire Line
	6950 1300 7000 1300
Connection ~ 6950 1200
Wire Wire Line
	6950 1400 7000 1400
Connection ~ 6950 1300
Wire Wire Line
	6950 1500 7000 1500
Connection ~ 6950 1400
Wire Wire Line
	6950 1600 7000 1600
Connection ~ 6950 1500
Wire Wire Line
	6950 1700 7000 1700
Connection ~ 6950 1600
Wire Wire Line
	6950 1800 7000 1800
Connection ~ 6950 1700
Wire Wire Line
	6950 1900 7000 1900
Connection ~ 6950 1800
Wire Wire Line
	6950 2000 7000 2000
Connection ~ 6950 1900
Wire Wire Line
	6950 2100 7000 2100
Connection ~ 6950 2000
Wire Wire Line
	6950 2200 7000 2200
Connection ~ 6950 2100
Wire Wire Line
	6950 2300 7000 2300
Connection ~ 6950 2200
Wire Wire Line
	6950 2400 7000 2400
Connection ~ 6950 2300
Wire Wire Line
	6950 2500 7000 2500
Connection ~ 6950 2400
Wire Wire Line
	6950 2600 7000 2600
Connection ~ 6950 2500
Wire Wire Line
	6950 2700 7000 2700
Connection ~ 6950 2600
Wire Wire Line
	6950 2800 7000 2800
Connection ~ 6950 2700
Wire Wire Line
	6950 2900 7000 2900
Connection ~ 6950 2800
Wire Wire Line
	6950 3000 7000 3000
Connection ~ 6950 2900
Wire Wire Line
	6950 3100 7000 3100
Connection ~ 6950 3000
Wire Wire Line
	6950 3200 7000 3200
Connection ~ 6950 3100
Wire Wire Line
	7000 5000 6950 5000
Wire Wire Line
	6950 5000 6950 5500
Wire Wire Line
	6950 5100 7000 5100
Wire Wire Line
	6950 5200 7000 5200
Connection ~ 6950 5100
Wire Wire Line
	6950 5300 7000 5300
Connection ~ 6950 5200
Wire Wire Line
	6950 5400 7000 5400
Connection ~ 6950 5300
Wire Wire Line
	6950 5500 7000 5500
Connection ~ 6950 5400
Wire Wire Line
	7000 5600 6950 5600
Wire Wire Line
	6950 5600 6950 6100
Wire Wire Line
	6950 5700 7000 5700
Wire Wire Line
	6950 5800 7000 5800
Connection ~ 6950 5700
Wire Wire Line
	6950 5900 7000 5900
Connection ~ 6950 5800
Wire Wire Line
	6950 6000 7000 6000
Connection ~ 6950 5900
Wire Wire Line
	6950 6100 7000 6100
Connection ~ 6950 6000
Wire Wire Line
	8600 700  8650 700 
Wire Wire Line
	8650 700  8650 5750
Wire Wire Line
	8650 800  8600 800 
Wire Wire Line
	8650 900  8600 900 
Connection ~ 8650 800 
Wire Wire Line
	8650 1000 8600 1000
Connection ~ 8650 900 
Wire Wire Line
	8650 1100 8600 1100
Connection ~ 8650 1000
Wire Wire Line
	8600 1200 8650 1200
Wire Wire Line
	8600 1300 8650 1300
Wire Wire Line
	8600 1400 8650 1400
Wire Wire Line
	8600 1500 8650 1500
Wire Wire Line
	8600 1600 8650 1600
Wire Wire Line
	8600 1700 8650 1700
Wire Wire Line
	8600 1800 8650 1800
Wire Wire Line
	8600 1900 8650 1900
Wire Wire Line
	8600 2000 8650 2000
Wire Wire Line
	8600 2100 8650 2100
Wire Wire Line
	8600 2200 8650 2200
Wire Wire Line
	8600 2300 8650 2300
Wire Wire Line
	8600 2400 8650 2400
Wire Wire Line
	8600 2500 8650 2500
Wire Wire Line
	8600 2600 8650 2600
Wire Wire Line
	8600 2700 8650 2700
Wire Wire Line
	8600 2800 8650 2800
Wire Wire Line
	8600 2900 8650 2900
Wire Wire Line
	8600 3000 8650 3000
Wire Wire Line
	8600 3100 8650 3100
Wire Wire Line
	8600 3200 8650 3200
Wire Wire Line
	8600 3300 8650 3300
Wire Wire Line
	8600 3400 8650 3400
Wire Wire Line
	8600 3500 8650 3500
Wire Wire Line
	8600 3600 8650 3600
Wire Wire Line
	8600 3700 8650 3700
Wire Wire Line
	8600 3800 8650 3800
Wire Wire Line
	8600 3900 8650 3900
Wire Wire Line
	8600 4000 8650 4000
Wire Wire Line
	8600 4100 8650 4100
Wire Wire Line
	8600 4200 8650 4200
Wire Wire Line
	8600 4300 8650 4300
Wire Wire Line
	8600 4400 8650 4400
Wire Wire Line
	8600 4500 8650 4500
Wire Wire Line
	8600 4600 8650 4600
Wire Wire Line
	8600 4700 8650 4700
Wire Wire Line
	8600 4800 8650 4800
Wire Wire Line
	8600 4900 8650 4900
Wire Wire Line
	8600 5000 8650 5000
Wire Wire Line
	8600 5100 8650 5100
Wire Wire Line
	8600 5200 8650 5200
Wire Wire Line
	8600 5300 8650 5300
Wire Wire Line
	8600 5400 8650 5400
Wire Wire Line
	8600 5500 8650 5500
Wire Wire Line
	8600 5600 8650 5600
Wire Wire Line
	8650 5700 8600 5700
Connection ~ 8650 1100
Connection ~ 8650 1200
Connection ~ 8650 1300
Connection ~ 8650 1400
Connection ~ 8650 1500
Connection ~ 8650 1600
Connection ~ 8650 1700
Connection ~ 8650 1800
Connection ~ 8650 1900
Connection ~ 8650 2000
Connection ~ 8650 2100
Connection ~ 8650 2200
Connection ~ 8650 2300
Connection ~ 8650 2400
Connection ~ 8650 2500
Connection ~ 8650 2600
Connection ~ 8650 2700
Connection ~ 8650 2800
Connection ~ 8650 2900
Connection ~ 8650 3000
Connection ~ 8650 3100
Connection ~ 8650 3200
Connection ~ 8650 3300
Connection ~ 8650 3400
Connection ~ 8650 3500
Connection ~ 8650 3600
Connection ~ 8650 3700
Connection ~ 8650 3800
Connection ~ 8650 3900
Connection ~ 8650 4000
Connection ~ 8650 4100
Connection ~ 8650 4200
Connection ~ 8650 4300
Connection ~ 8650 4400
Connection ~ 8650 4500
Connection ~ 8650 4600
Connection ~ 8650 4700
Connection ~ 8650 4800
Connection ~ 8650 4900
Connection ~ 8650 5000
Connection ~ 8650 5100
Connection ~ 8650 5200
Connection ~ 8650 5300
Connection ~ 8650 5400
Connection ~ 8650 5500
Connection ~ 8650 5600
Wire Wire Line
	11000 700  11050 700 
Wire Wire Line
	11000 800  11050 800 
Wire Wire Line
	11000 900  11050 900 
Wire Wire Line
	11000 1000 11050 1000
Wire Wire Line
	11000 1100 11050 1100
Wire Wire Line
	11000 1200 11050 1200
Wire Wire Line
	11000 1300 11050 1300
Wire Wire Line
	11000 1400 11050 1400
Wire Wire Line
	11000 1500 11050 1500
Wire Wire Line
	11000 1600 11050 1600
Wire Wire Line
	11000 1700 11050 1700
Wire Wire Line
	11000 1800 11050 1800
Wire Wire Line
	11000 1900 11050 1900
Wire Wire Line
	11000 2000 11050 2000
Wire Wire Line
	11000 2100 11050 2100
Wire Wire Line
	11000 2200 11050 2200
Wire Wire Line
	11000 2300 11050 2300
Wire Wire Line
	11000 2400 11050 2400
Wire Wire Line
	11000 2500 11050 2500
Wire Wire Line
	11000 2600 11050 2600
Wire Wire Line
	11000 2700 11050 2700
Wire Wire Line
	11000 2800 11050 2800
Wire Wire Line
	11000 2900 11050 2900
Wire Wire Line
	11000 3000 11050 3000
Wire Wire Line
	11000 3100 11050 3100
Wire Wire Line
	11000 3200 11050 3200
Wire Wire Line
	11000 3300 11050 3300
Wire Wire Line
	11000 3400 11050 3400
Wire Wire Line
	11000 3500 11050 3500
Wire Wire Line
	11000 3600 11050 3600
Wire Wire Line
	11000 3700 11050 3700
Wire Wire Line
	11000 3800 11050 3800
Wire Wire Line
	11000 3900 11050 3900
Wire Wire Line
	11000 4000 11050 4000
Wire Wire Line
	11000 4100 11050 4100
Wire Wire Line
	11000 4200 11050 4200
Wire Wire Line
	11000 4300 11050 4300
Wire Wire Line
	11000 4400 11050 4400
Wire Wire Line
	11000 4500 11050 4500
Wire Wire Line
	11000 4600 11050 4600
Wire Wire Line
	11000 4700 11050 4700
Wire Wire Line
	11000 4800 11050 4800
Wire Wire Line
	11000 4900 11050 4900
Wire Wire Line
	11000 5000 11050 5000
Wire Wire Line
	11000 5100 11050 5100
Wire Wire Line
	11000 5200 11050 5200
Wire Wire Line
	11000 5300 11050 5300
Wire Wire Line
	11000 5400 11050 5400
Wire Wire Line
	11000 5500 11050 5500
Wire Wire Line
	11000 5600 11050 5600
Wire Wire Line
	11050 700  11050 5750
Connection ~ 11050 800 
Connection ~ 11050 900 
Connection ~ 11050 1000
Connection ~ 11050 1100
Connection ~ 11050 1200
Connection ~ 11050 1300
Connection ~ 11050 1400
Connection ~ 11050 1500
Connection ~ 11050 1600
Connection ~ 11050 1700
Connection ~ 11050 1800
Connection ~ 11050 1900
Connection ~ 11050 2000
Connection ~ 11050 2100
Connection ~ 11050 2200
Connection ~ 11050 2300
Connection ~ 11050 2400
Connection ~ 11050 2500
Connection ~ 11050 2600
Connection ~ 11050 2700
Connection ~ 11050 2800
Connection ~ 11050 2900
Connection ~ 11050 3000
Connection ~ 11050 3100
Connection ~ 11050 3200
Connection ~ 11050 3300
Connection ~ 11050 3400
Connection ~ 11050 3500
Connection ~ 11050 3600
Connection ~ 11050 3700
Connection ~ 11050 3800
Connection ~ 11050 3900
Connection ~ 11050 4000
Connection ~ 11050 4100
Connection ~ 11050 4200
Connection ~ 11050 4300
Connection ~ 11050 4400
Connection ~ 11050 4500
Connection ~ 11050 4600
Connection ~ 11050 4700
Connection ~ 11050 4800
Connection ~ 11050 4900
Connection ~ 11050 5000
Connection ~ 11050 5100
Connection ~ 11050 5200
Connection ~ 11050 5300
Connection ~ 11050 5400
Connection ~ 11050 5500
NoConn ~ 9400 700 
NoConn ~ 9400 800 
NoConn ~ 9400 900 
NoConn ~ 9400 1000
$Comp
L GND #PWR?
U 1 1 58D81456
P 8650 5750
F 0 "#PWR?" H 8650 5500 50  0001 C CNN
F 1 "GND" H 8650 5600 50  0001 C CNN
F 2 "" H 8650 5750 50  0000 C CNN
F 3 "" H 8650 5750 50  0000 C CNN
	1    8650 5750
	1    0    0    -1  
$EndComp
Connection ~ 8650 5700
$Comp
L GND #PWR?
U 1 1 58D815E1
P 11050 5750
F 0 "#PWR?" H 11050 5500 50  0001 C CNN
F 1 "GND" H 11050 5600 50  0001 C CNN
F 2 "" H 11050 5750 50  0000 C CNN
F 3 "" H 11050 5750 50  0000 C CNN
	1    11050 5750
	1    0    0    -1  
$EndComp
Connection ~ 11050 5600
$EndSCHEMATC
