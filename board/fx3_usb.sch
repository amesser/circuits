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
Wire Wire Line
	10650 4750 10150 4750
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
F 0 "U?" H 8500 6000 60  0000 C CNN
F 1 "5CEFA2F23" H 9000 1100 60  0000 C CNN
F 2 "" H 9000 3700 60  0001 C CNN
F 3 "" H 9000 3700 60  0001 C CNN
	1    9000 3600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
