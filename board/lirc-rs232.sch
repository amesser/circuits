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
LIBS:lirc-rs232-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RS232 IR Receiver"
Date "14 feb 2015"
Rev "1"
Comp "Copyright (c) 2015 Andreas Messer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R108
U 1 1 54DE4566
P 6000 650
F 0 "R108" V 6080 650 40  0000 C CNN
F 1 "100k" V 6007 651 40  0000 C CNN
F 2 "~" V 5930 650 30  0000 C CNN
F 3 "~" H 6000 650 30  0000 C CNN
	1    6000 650 
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR110
U 1 1 54DE4575
P 4150 1150
F 0 "#PWR110" H 4150 1150 30  0001 C CNN
F 1 "GND" H 4150 1080 30  0001 C CNN
F 2 "" H 4150 1150 60  0000 C CNN
F 3 "" H 4150 1150 60  0000 C CNN
	1    4150 1150
	-1   0    0    -1  
$EndComp
$Comp
L C C107
U 1 1 54DE4584
P 6750 1250
F 0 "C107" H 6750 1350 40  0000 L CNN
F 1 "100n" H 6756 1165 40  0000 L CNN
F 2 "~" H 6788 1100 30  0000 C CNN
F 3 "~" H 6750 1250 60  0000 C CNN
	1    6750 1250
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR113
U 1 1 54DE4591
P 6750 1500
F 0 "#PWR113" H 6750 1500 30  0001 C CNN
F 1 "GND" H 6750 1430 30  0001 C CNN
F 2 "" H 6750 1500 60  0000 C CNN
F 3 "" H 6750 1500 60  0000 C CNN
	1    6750 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6750 1500 6750 1450
$Comp
L C C105
U 1 1 54DE45BC
P 4150 900
F 0 "C105" H 4150 1000 40  0000 L CNN
F 1 "100n" H 4156 815 40  0000 L CNN
F 2 "~" H 4188 750 30  0000 C CNN
F 3 "~" H 4150 900 60  0000 C CNN
	1    4150 900 
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR112
U 1 1 54DE463B
P 5450 2850
F 0 "#PWR112" H 5450 2850 30  0001 C CNN
F 1 "GND" H 5450 2780 30  0001 C CNN
F 2 "" H 5450 2850 60  0000 C CNN
F 3 "" H 5450 2850 60  0000 C CNN
	1    5450 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6750 650  6750 1050
Connection ~ 6750 1000
$Comp
L DB9 J101
U 1 1 54DE4698
P 750 1350
F 0 "J101" H 750 1900 70  0000 C CNN
F 1 "DB9" H 750 800 70  0000 C CNN
F 2 "~" H 750 1350 60  0000 C CNN
F 3 "~" H 750 1350 60  0000 C CNN
	1    750  1350
	-1   0    0    1   
$EndComp
$Comp
L DIODE D103
U 1 1 54DE46A7
P 1500 900
F 0 "D103" H 1500 1000 40  0000 C CNN
F 1 "DIODE" H 1500 800 40  0000 C CNN
F 2 "~" H 1500 900 60  0000 C CNN
F 3 "~" H 1500 900 60  0000 C CNN
	1    1500 900 
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D101
U 1 1 54DE46B4
P 1300 900
F 0 "D101" H 1300 1000 40  0000 C CNN
F 1 "DIODE" H 1300 800 40  0000 C CNN
F 2 "~" H 1300 900 60  0000 C CNN
F 3 "~" H 1300 900 60  0000 C CNN
	1    1300 900 
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D102
U 1 1 54DE46BA
P 1400 1800
F 0 "D102" H 1400 1900 40  0000 C CNN
F 1 "DIODE" H 1400 1700 40  0000 C CNN
F 2 "~" H 1400 1800 60  0000 C CNN
F 3 "~" H 1400 1800 60  0000 C CNN
	1    1400 1800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1200 1250 1300 1250
Wire Wire Line
	1300 1250 1300 1100
Wire Wire Line
	1200 1550 1500 1550
Wire Wire Line
	1500 1550 1500 1100
Wire Wire Line
	1200 1350 1400 1350
Wire Wire Line
	1400 1350 1400 1600
$Comp
L GND #PWR101
U 1 1 54DE4708
P 1250 1800
F 0 "#PWR101" H 1250 1800 30  0001 C CNN
F 1 "GND" H 1250 1730 30  0001 C CNN
F 2 "" H 1250 1800 60  0000 C CNN
F 3 "" H 1250 1800 60  0000 C CNN
	1    1250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1750 1250 1750
Wire Wire Line
	1250 1750 1250 1800
Wire Wire Line
	1300 700  1300 650 
Wire Wire Line
	1300 650  1550 650 
Wire Wire Line
	1500 650  1500 700 
$Comp
L R R101
U 1 1 54DE4735
P 1800 650
F 0 "R101" V 1880 650 40  0000 C CNN
F 1 "100" V 1807 651 40  0000 C CNN
F 2 "~" V 1730 650 30  0000 C CNN
F 3 "~" H 1800 650 30  0000 C CNN
	1    1800 650 
	0    1    1    0   
$EndComp
Connection ~ 1500 650 
$Comp
L R R102
U 1 1 54DE4757
P 1800 2050
F 0 "R102" V 1880 2050 40  0000 C CNN
F 1 "100" V 1807 2051 40  0000 C CNN
F 2 "~" V 1730 2050 30  0000 C CNN
F 3 "~" H 1800 2050 30  0000 C CNN
	1    1800 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 2000 1400 2050
Wire Wire Line
	1400 2050 1550 2050
$Comp
L ZENER D104
U 1 1 54DE477A
P 2100 900
F 0 "D104" H 2100 1000 50  0000 C CNN
F 1 "12V" H 2100 800 40  0000 C CNN
F 2 "~" H 2100 900 60  0000 C CNN
F 3 "~" H 2100 900 60  0000 C CNN
	1    2100 900 
	0    -1   -1   0   
$EndComp
$Comp
L ZENER D105
U 1 1 54DE4787
P 2100 1800
F 0 "D105" H 2100 1900 50  0000 C CNN
F 1 "12V" H 2100 1700 40  0000 C CNN
F 2 "~" H 2100 1800 60  0000 C CNN
F 3 "~" H 2100 1800 60  0000 C CNN
	1    2100 1800
	0    -1   -1   0   
$EndComp
$Comp
L CP C101
U 1 1 54DE478F
P 2300 900
F 0 "C101" H 2350 1000 40  0000 L CNN
F 1 "47µ" H 2350 800 40  0000 L CNN
F 2 "~" H 2400 750 30  0000 C CNN
F 3 "~" H 2300 900 300 0000 C CNN
	1    2300 900 
	1    0    0    -1  
$EndComp
$Comp
L CP C102
U 1 1 54DE47A1
P 2300 1800
F 0 "C102" H 2350 1900 40  0000 L CNN
F 1 "47µ" H 2350 1700 40  0000 L CNN
F 2 "~" H 2400 1650 30  0000 C CNN
F 3 "~" H 2300 1800 300 0000 C CNN
	1    2300 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 650  2300 700 
Wire Wire Line
	2050 650  2350 650 
Wire Wire Line
	2100 650  2100 700 
Connection ~ 2100 650 
Wire Wire Line
	2050 2050 3450 2050
Wire Wire Line
	2100 2050 2100 2000
Wire Wire Line
	2300 2050 2300 2000
Connection ~ 2100 2050
$Comp
L GND #PWR102
U 1 1 54DE4848
P 2100 1150
F 0 "#PWR102" H 2100 1150 30  0001 C CNN
F 1 "GND" H 2100 1080 30  0001 C CNN
F 2 "" H 2100 1150 60  0000 C CNN
F 3 "" H 2100 1150 60  0000 C CNN
	1    2100 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR104
U 1 1 54DE484E
P 2300 1150
F 0 "#PWR104" H 2300 1150 30  0001 C CNN
F 1 "GND" H 2300 1080 30  0001 C CNN
F 2 "" H 2300 1150 60  0000 C CNN
F 3 "" H 2300 1150 60  0000 C CNN
	1    2300 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1100 2100 1150
Wire Wire Line
	2300 1100 2300 1150
$Comp
L GND #PWR103
U 1 1 54DE4899
P 2100 1550
F 0 "#PWR103" H 2100 1550 30  0001 C CNN
F 1 "GND" H 2100 1480 30  0001 C CNN
F 2 "" H 2100 1550 60  0000 C CNN
F 3 "" H 2100 1550 60  0000 C CNN
	1    2100 1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 1550 2100 1600
$Comp
L GND #PWR105
U 1 1 54DE48C3
P 2300 1550
F 0 "#PWR105" H 2300 1550 30  0001 C CNN
F 1 "GND" H 2300 1480 30  0001 C CNN
F 2 "" H 2300 1550 60  0000 C CNN
F 3 "" H 2300 1550 60  0000 C CNN
	1    2300 1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 1550 2300 1600
$Comp
L LM7805 U101
U 1 1 54DE48F0
P 2750 700
F 0 "U101" H 2900 504 60  0000 C CNN
F 1 "LM78L05" H 2750 900 60  0000 C CNN
F 2 "~" H 2750 700 60  0000 C CNN
F 3 "~" H 2750 700 60  0000 C CNN
	1    2750 700 
	1    0    0    -1  
$EndComp
Connection ~ 2300 650 
$Comp
L GND #PWR106
U 1 1 54DE4923
P 2750 1000
F 0 "#PWR106" H 2750 1000 30  0001 C CNN
F 1 "GND" H 2750 930 30  0001 C CNN
F 2 "" H 2750 1000 60  0000 C CNN
F 3 "" H 2750 1000 60  0000 C CNN
	1    2750 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 950  2750 1000
$Comp
L CP C104
U 1 1 54DE4955
P 3200 900
F 0 "C104" H 3250 1000 40  0000 L CNN
F 1 "10u" H 3250 800 40  0000 L CNN
F 2 "~" H 3300 750 30  0000 C CNN
F 3 "~" H 3200 900 300 0000 C CNN
	1    3200 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 650  3200 700 
$Comp
L GND #PWR109
U 1 1 54DE4988
P 3200 1150
F 0 "#PWR109" H 3200 1150 30  0001 C CNN
F 1 "GND" H 3200 1080 30  0001 C CNN
F 2 "" H 3200 1150 60  0000 C CNN
F 3 "" H 3200 1150 60  0000 C CNN
	1    3200 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1100 3200 1150
$Comp
L PNP Q101
U 1 1 54DE49BA
P 3550 1050
F 0 "Q101" H 3550 900 60  0000 R CNN
F 1 "PNP" H 3550 1200 60  0000 R CNN
F 2 "~" H 3550 1050 60  0000 C CNN
F 3 "~" H 3550 1050 60  0000 C CNN
	1    3550 1050
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 650  3450 850 
Connection ~ 3200 650 
$Comp
L R R105
U 1 1 54DE49F8
P 3450 1700
F 0 "R105" V 3530 1700 40  0000 C CNN
F 1 "6k8" V 3457 1701 40  0000 C CNN
F 2 "~" V 3380 1700 30  0000 C CNN
F 3 "~" H 3450 1700 30  0000 C CNN
	1    3450 1700
	1    0    0    -1  
$EndComp
Connection ~ 2300 2050
$Comp
L R R103
U 1 1 54DE4A6F
P 3100 1350
F 0 "R103" V 3180 1350 40  0000 C CNN
F 1 "3k3" V 3107 1351 40  0000 C CNN
F 2 "~" V 3030 1350 30  0000 C CNN
F 3 "~" H 3100 1350 30  0000 C CNN
	1    3100 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3350 1350 3450 1350
Wire Wire Line
	1200 1150 1800 1150
Wire Wire Line
	1800 1150 1800 1350
Wire Wire Line
	1800 1350 2850 1350
Wire Wire Line
	3450 1250 3450 1450
$Comp
L R R106
U 1 1 54DE4B33
P 3800 1500
F 0 "R106" V 3880 1500 40  0000 C CNN
F 1 "23k" V 3807 1501 40  0000 C CNN
F 2 "~" V 3730 1500 30  0000 C CNN
F 3 "~" H 3800 1500 30  0000 C CNN
	1    3800 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 2050 3450 1950
Connection ~ 3450 1350
Wire Wire Line
	3750 1050 3800 1050
Connection ~ 3450 650 
Wire Wire Line
	3800 1050 3800 1250
$Comp
L C C106
U 1 1 54DE4CA9
P 4200 2750
F 0 "C106" H 4200 2850 40  0000 L CNN
F 1 "100p" H 4206 2665 40  0000 L CNN
F 2 "~" H 4238 2600 30  0000 C CNN
F 3 "~" H 4200 2750 60  0000 C CNN
	1    4200 2750
	1    0    0    1   
$EndComp
Wire Wire Line
	6750 650  6250 650 
Wire Wire Line
	6600 1000 6750 1000
$Comp
L CONN_3 IC101
U 1 1 54DE4E33
P 2500 2400
F 0 "IC101" V 2450 2400 50  0000 C CNN
F 1 "TSSOP" V 2550 2400 40  0000 C CNN
F 2 "~" H 2500 2400 60  0000 C CNN
F 3 "~" H 2500 2400 60  0000 C CNN
	1    2500 2400
	-1   0    0    -1  
$EndComp
$Comp
L R R104
U 1 1 54DE4E87
P 3400 2300
F 0 "R104" V 3480 2300 40  0000 C CNN
F 1 "100" V 3407 2301 40  0000 C CNN
F 2 "~" V 3330 2300 30  0000 C CNN
F 3 "~" H 3400 2300 30  0000 C CNN
	1    3400 2300
	0    1    -1   0   
$EndComp
Wire Wire Line
	3100 2300 3100 2350
Connection ~ 3100 2300
Wire Wire Line
	2850 2300 3150 2300
$Comp
L R R107
U 1 1 54DE51A9
P 3900 2500
F 0 "R107" V 3980 2500 40  0000 C CNN
F 1 "470" V 3907 2501 40  0000 C CNN
F 2 "~" V 3830 2500 30  0000 C CNN
F 3 "~" H 3900 2500 30  0000 C CNN
	1    3900 2500
	0    -1   1    0   
$EndComp
Wire Wire Line
	4700 2550 5050 2550
Connection ~ 4700 2550
$Comp
L GND #PWR111
U 1 1 54DE533D
P 4200 3000
F 0 "#PWR111" H 4200 3000 30  0001 C CNN
F 1 "GND" H 4200 2930 30  0001 C CNN
F 2 "" H 4200 3000 60  0000 C CNN
F 3 "" H 4200 3000 60  0000 C CNN
	1    4200 3000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4200 2950 4200 3000
$Comp
L ATTINY2313-S IC102
U 1 1 54DE4556
P 5450 1800
F 0 "IC102" H 4500 2800 40  0000 C CNN
F 1 "ATTINY2313-S" H 6200 900 40  0000 C CNN
F 2 "SO20" H 5450 1800 35  0000 C CIN
F 3 "~" H 5450 1800 60  0000 C CNN
	1    5450 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 2000 4300 2000
Wire Wire Line
	3800 2000 3800 1750
Wire Wire Line
	3150 650  5750 650 
Wire Wire Line
	5450 650  5450 700 
Wire Wire Line
	5450 2800 5450 2850
Connection ~ 5450 650 
Wire Wire Line
	4150 1100 4150 1150
Wire Wire Line
	4150 2500 4300 2500
Wire Wire Line
	4200 2500 4200 2550
Connection ~ 4200 2500
Wire Wire Line
	4150 700  4150 650 
Connection ~ 4150 650 
$Comp
L GND #PWR108
U 1 1 54DE56F7
P 3100 2800
F 0 "#PWR108" H 3100 2800 30  0001 C CNN
F 1 "GND" H 3100 2730 30  0001 C CNN
F 2 "" H 3100 2800 60  0000 C CNN
F 3 "" H 3100 2800 60  0000 C CNN
	1    3100 2800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3100 2750 3100 2800
$Comp
L GND #PWR107
U 1 1 54DE5793
P 2950 2450
F 0 "#PWR107" H 2950 2450 30  0001 C CNN
F 1 "GND" H 2950 2380 30  0001 C CNN
F 2 "" H 2950 2450 60  0000 C CNN
F 3 "" H 2950 2450 60  0000 C CNN
	1    2950 2450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2850 2400 2950 2400
Wire Wire Line
	2950 2400 2950 2450
$Comp
L C C103
U 1 1 54DE4E6D
P 3100 2550
F 0 "C103" H 3100 2650 40  0000 L CNN
F 1 "100n" H 3106 2465 40  0000 L CNN
F 2 "~" H 3138 2400 30  0000 C CNN
F 3 "~" H 3100 2550 60  0000 C CNN
	1    3100 2550
	1    0    0    1   
$EndComp
Wire Wire Line
	2850 2500 2900 2500
Wire Wire Line
	2900 2500 2900 2900
Wire Wire Line
	2900 2900 3450 2900
Wire Wire Line
	3450 2900 3450 2500
Wire Wire Line
	3450 2500 3650 2500
Wire Wire Line
	3650 2300 3950 2300
Wire Wire Line
	3950 2300 3950 650 
Connection ~ 3950 650 
$EndSCHEMATC
