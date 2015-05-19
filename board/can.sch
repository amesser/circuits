EESchema Schematic File Version 2
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
LIBS:special
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
LIBS:amesser-com
LIBS:amesser-miscic
LIBS:amesser-display
LIBS:amesser-conn
LIBS:amesser-switch
LIBS:controller-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title "CAN Interface"
Date "19 may 2015"
Rev "1"
Comp "(c) 2015 Andreas Messer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCP2515-I/P U301
U 1 1 55576551
P 4200 4350
F 0 "U301" H 3800 5000 60  0000 C CNN
F 1 "MCP2515-I/P" H 3950 3700 60  0000 C CNN
F 2 "~" H 4200 4350 60  0000 C CNN
F 3 "~" H 4200 4350 60  0000 C CNN
	1    4200 4350
	1    0    0    -1  
$EndComp
$Comp
L C C301
U 1 1 555765BA
P 4450 3500
F 0 "C301" H 4450 3600 40  0000 L CNN
F 1 "100n" H 4456 3415 40  0000 L CNN
F 2 "~" H 4488 3350 30  0000 C CNN
F 3 "~" H 4450 3500 60  0000 C CNN
	1    4450 3500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR051
U 1 1 555765C0
P 4700 3550
F 0 "#PWR051" H 4700 3550 30  0001 C CNN
F 1 "GND" H 4700 3480 30  0001 C CNN
F 2 "" H 4700 3550 60  0000 C CNN
F 3 "" H 4700 3550 60  0000 C CNN
	1    4700 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR052
U 1 1 555765C6
P 4200 5200
F 0 "#PWR052" H 4200 5200 30  0001 C CNN
F 1 "GND" H 4200 5130 30  0001 C CNN
F 2 "" H 4200 5200 60  0000 C CNN
F 3 "" H 4200 5200 60  0000 C CNN
	1    4200 5200
	1    0    0    -1  
$EndComp
$Comp
L MCP2551-I/P IC301
U 1 1 55576A6F
P 5650 3450
F 0 "IC301" H 5400 3750 40  0000 C CNN
F 1 "MCP2551-I/P" H 5900 3150 40  0000 C CNN
F 2 "DIP8" H 5650 3450 35  0000 C CIN
F 3 "" H 5650 3450 60  0000 C CNN
	1    5650 3450
	1    0    0    -1  
$EndComp
$Comp
L C C303
U 1 1 55576A7C
P 5900 3000
F 0 "C303" H 5900 3100 40  0000 L CNN
F 1 "100n" H 5906 2915 40  0000 L CNN
F 2 "~" H 5938 2850 30  0000 C CNN
F 3 "~" H 5900 3000 60  0000 C CNN
	1    5900 3000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR053
U 1 1 55576A82
P 6150 3050
F 0 "#PWR053" H 6150 3050 30  0001 C CNN
F 1 "GND" H 6150 2980 30  0001 C CNN
F 2 "" H 6150 3050 60  0000 C CNN
F 3 "" H 6150 3050 60  0000 C CNN
	1    6150 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR054
U 1 1 55576A8E
P 5650 3900
F 0 "#PWR054" H 5650 3900 30  0001 C CNN
F 1 "GND" H 5650 3830 30  0001 C CNN
F 2 "" H 5650 3900 60  0000 C CNN
F 3 "" H 5650 3900 60  0000 C CNN
	1    5650 3900
	1    0    0    -1  
$EndComp
$Comp
L CONN_3X2 P302
U 1 1 55576BCE
P 6150 5100
F 0 "P302" H 6150 5350 50  0000 C CNN
F 1 "CONN_3X2" V 6150 5150 40  0000 C CNN
F 2 "" H 6150 5100 60  0000 C CNN
F 3 "" H 6150 5100 60  0000 C CNN
	1    6150 5100
	1    0    0    -1  
$EndComp
$Comp
L R R302
U 1 1 55576BDB
P 5450 4950
F 0 "R302" V 5530 4950 40  0000 C CNN
F 1 "10k" V 5457 4951 40  0000 C CNN
F 2 "~" V 5380 4950 30  0000 C CNN
F 3 "~" H 5450 4950 30  0000 C CNN
	1    5450 4950
	0    -1   -1   0   
$EndComp
$Comp
L R R303
U 1 1 55576BEB
P 5450 5050
F 0 "R303" V 5530 5050 40  0000 C CNN
F 1 "10k" V 5457 5051 40  0000 C CNN
F 2 "~" V 5380 5050 30  0000 C CNN
F 3 "~" H 5450 5050 30  0000 C CNN
	1    5450 5050
	0    -1   -1   0   
$EndComp
$Comp
L R R304
U 1 1 55576BF1
P 5450 5150
F 0 "R304" V 5530 5150 40  0000 C CNN
F 1 "10k" V 5457 5151 40  0000 C CNN
F 2 "~" V 5380 5150 30  0000 C CNN
F 3 "~" H 5450 5150 30  0000 C CNN
	1    5450 5150
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR055
U 1 1 55576E88
P 6600 5200
F 0 "#PWR055" H 6600 5200 30  0001 C CNN
F 1 "GND" H 6600 5130 30  0001 C CNN
F 2 "" H 6600 5200 60  0000 C CNN
F 3 "" H 6600 5200 60  0000 C CNN
	1    6600 5200
	1    0    0    -1  
$EndComp
$Comp
L C C304
U 1 1 55577375
P 7250 4100
F 0 "C304" H 7250 4200 40  0000 L CNN
F 1 "100n" H 7256 4015 40  0000 L CNN
F 2 "~" H 7288 3950 30  0000 C CNN
F 3 "~" H 7250 4100 60  0000 C CNN
	1    7250 4100
	1    0    0    -1  
$EndComp
$Comp
L R R305
U 1 1 5557737D
P 6600 3850
F 0 "R305" V 6680 3850 40  0000 C CNN
F 1 "60" V 6607 3851 40  0000 C CNN
F 2 "~" V 6530 3850 30  0000 C CNN
F 3 "~" H 6600 3850 30  0000 C CNN
	1    6600 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR056
U 1 1 5557741C
P 5700 4750
F 0 "#PWR056" H 5700 4750 30  0001 C CNN
F 1 "GND" H 5700 4680 30  0001 C CNN
F 2 "" H 5700 4750 60  0000 C CNN
F 3 "" H 5700 4750 60  0000 C CNN
	1    5700 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR057
U 1 1 555774A7
P 7250 4350
F 0 "#PWR057" H 7250 4350 30  0001 C CNN
F 1 "GND" H 7250 4280 30  0001 C CNN
F 2 "" H 7250 4350 60  0000 C CNN
F 3 "" H 7250 4350 60  0000 C CNN
	1    7250 4350
	1    0    0    -1  
$EndComp
$Comp
L R R301
U 1 1 55577153
P 5150 4000
F 0 "R301" V 5230 4000 40  0000 C CNN
F 1 "1k" V 5157 4001 40  0000 C CNN
F 2 "~" V 5080 4000 30  0000 C CNN
F 3 "~" H 5150 4000 30  0000 C CNN
	1    5150 4000
	1    0    0    -1  
$EndComp
$Comp
L DB9 J301
U 1 1 555A3322
P 7750 3450
F 0 "J301" H 7750 4000 70  0000 C CNN
F 1 "DB9" H 7750 2900 70  0000 C CNN
F 2 "" H 7750 3450 60  0000 C CNN
F 3 "" H 7750 3450 60  0000 C CNN
	1    7750 3450
	1    0    0    1   
$EndComp
$Comp
L GND #PWR058
U 1 1 555A3540
P 7150 3500
F 0 "#PWR058" H 7150 3500 30  0001 C CNN
F 1 "GND" H 7150 3430 30  0001 C CNN
F 2 "" H 7150 3500 60  0000 C CNN
F 3 "" H 7150 3500 60  0000 C CNN
	1    7150 3500
	1    0    0    -1  
$EndComp
$Comp
L R R306
U 1 1 555A3612
P 6700 3850
F 0 "R306" V 6780 3850 40  0000 C CNN
F 1 "60" V 6707 3851 40  0000 C CNN
F 2 "~" V 6630 3850 30  0000 C CNN
F 3 "~" H 6700 3850 30  0000 C CNN
	1    6700 3850
	1    0    0    -1  
$EndComp
$Comp
L CONN_2X2 P301
U 1 1 555A3642
P 6150 4200
F 0 "P301" H 6150 4350 50  0000 C CNN
F 1 "CONN_2X2" H 6160 4070 40  0000 C CNN
F 2 "" H 6150 4200 60  0000 C CNN
F 3 "" H 6150 4200 60  0000 C CNN
	1    6150 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 5200 4200 5150
Wire Wire Line
	4200 3000 4200 3550
Wire Wire Line
	4200 3500 4250 3500
Wire Wire Line
	4650 3500 4700 3500
Wire Wire Line
	4700 3500 4700 3550
Connection ~ 4200 3500
Wire Wire Line
	3250 3850 3500 3850
Wire Wire Line
	3250 3000 5700 3000
Connection ~ 5650 3000
Wire Wire Line
	6100 3000 6150 3000
Wire Wire Line
	6150 3000 6150 3050
Wire Wire Line
	5700 4950 5750 4950
Wire Wire Line
	5700 5050 5750 5050
Wire Wire Line
	5700 5150 5750 5150
Wire Wire Line
	5650 3900 5650 3850
Wire Wire Line
	5150 3650 5200 3650
Wire Wire Line
	4900 4750 5150 4750
Wire Wire Line
	6550 4950 6600 4950
Wire Wire Line
	6600 4950 6600 5200
Wire Wire Line
	6550 5150 6600 5150
Connection ~ 6600 5150
Wire Wire Line
	6550 5050 6600 5050
Connection ~ 6600 5050
Wire Wire Line
	5150 3650 5150 3750
Wire Wire Line
	5150 4750 5150 4250
Wire Wire Line
	5200 3250 5050 3250
Wire Wire Line
	5050 3250 5050 4150
Wire Wire Line
	5050 4150 4900 4150
Wire Wire Line
	5200 3350 5000 3350
Wire Wire Line
	5000 3350 5000 4050
Wire Wire Line
	5000 4050 4900 4050
Wire Wire Line
	3250 4750 3500 4750
Wire Wire Line
	3250 4250 3500 4250
Wire Wire Line
	3250 4350 3500 4350
Wire Wire Line
	3250 4450 3500 4450
Wire Wire Line
	3250 4550 3500 4550
Wire Wire Line
	5650 3000 5650 3050
Wire Wire Line
	7150 3150 7300 3150
Wire Wire Line
	7300 3850 7250 3850
Wire Wire Line
	7250 3850 7250 3900
Wire Wire Line
	7250 4300 7250 4350
Wire Wire Line
	7150 3150 7150 3500
Wire Wire Line
	6100 3350 7300 3350
Wire Wire Line
	7300 3450 7150 3450
Connection ~ 7150 3450
Wire Wire Line
	7300 3250 6950 3250
Wire Wire Line
	6950 3250 6950 3550
Wire Wire Line
	6950 3550 6100 3550
Wire Wire Line
	4900 4350 5100 4350
Wire Wire Line
	5100 4350 5100 4950
Wire Wire Line
	5100 4950 5200 4950
Wire Wire Line
	5200 5050 5050 5050
Wire Wire Line
	5050 5050 5050 4450
Wire Wire Line
	5050 4450 4900 4450
Wire Wire Line
	4900 4550 5000 4550
Wire Wire Line
	5000 4550 5000 5150
Wire Wire Line
	5000 5150 5200 5150
Wire Wire Line
	6600 3600 6600 3550
Connection ~ 6600 3550
Wire Wire Line
	6700 3600 6700 3350
Connection ~ 6700 3350
Wire Wire Line
	6600 4100 6600 4150
Wire Wire Line
	6600 4150 6550 4150
Wire Wire Line
	6700 4100 6700 4250
Wire Wire Line
	6700 4250 6550 4250
$Comp
L C C302
U 1 1 555A3953
P 5700 4500
F 0 "C302" H 5700 4600 40  0000 L CNN
F 1 "100n" H 5706 4415 40  0000 L CNN
F 2 "~" H 5738 4350 30  0000 C CNN
F 3 "~" H 5700 4500 60  0000 C CNN
	1    5700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4150 5700 4150
Wire Wire Line
	5700 4150 5700 4300
Wire Wire Line
	5750 4250 5700 4250
Connection ~ 5700 4250
Wire Wire Line
	5700 4700 5700 4750
Text HLabel 3250 3000 0    60   Input ~ 0
+5V
Text HLabel 3250 2800 0    60   Output ~ 0
CAN Power
Wire Wire Line
	7250 3750 7300 3750
Wire Wire Line
	7250 2800 7250 3750
Wire Wire Line
	7250 2800 3250 2800
Connection ~ 4200 3000
Text HLabel 3250 3850 0    60   Input ~ 0
RESET
Text HLabel 3250 4050 0    60   Output ~ 0
INT
Wire Wire Line
	3500 4050 3250 4050
Text HLabel 3250 4250 0    60   Input ~ 0
CS
Text HLabel 3250 4350 0    60   Input ~ 0
SCK
Text HLabel 3250 4450 0    60   Input ~ 0
SI
Text HLabel 3250 4550 0    60   Output ~ 0
SO
Text HLabel 3250 4750 0    60   Input ~ 0
OSC1/CLK
Text HLabel 3250 4850 0    60   Output ~ 0
OSC2
Wire Wire Line
	3500 4850 3250 4850
NoConn ~ 4900 3850
NoConn ~ 4900 4850
NoConn ~ 5200 3500
NoConn ~ 7300 3050
NoConn ~ 7300 3550
NoConn ~ 7300 3650
$EndSCHEMATC
