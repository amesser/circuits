EESchema Schematic File Version 2  date Sa 09 Mär 2013 20:26:20 CET
LIBS:amesser-power
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
LIBS:amesser-ad
LIBS:amesser-discrete
LIBS:amesser-linear
LIBS:amesser-conn
LIBS:sdlogger-attiny2313-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title "SD Card Data Logger"
Date "9 mar 2013"
Rev "1.1"
Comp "Copyright (c) 2013 Andreas Messer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L -1.5V #PWR01
U 1 1 513B87C4
P 3000 1400
F 0 "#PWR01" H 3000 1350 20  0001 C CNN
F 1 "-1.5V" H 3000 1500 30  0000 C CNN
	1    3000 1400
	-1   0    0    1   
$EndComp
$Comp
L LT1585CM U3
U 1 1 513B6FE5
P 5750 6000
F 0 "U3" H 5750 6300 60  0000 C CNN
F 1 "AS1117-3.3" H 5750 6000 50  0000 C CNN
	1    5750 6000
	1    0    0    -1  
$EndComp
Wire Notes Line
	9350 5650 9350 6200
Wire Notes Line
	8850 5650 8850 6200
Wire Notes Line
	7700 5650 7700 6200
Wire Notes Line
	4800 5650 6700 5650
Wire Notes Line
	4800 5650 4800 6550
Wire Notes Line
	4800 6550 6700 6550
Wire Notes Line
	6700 6550 6700 5650
Wire Bus Line
	4850 1950 4850 900 
Wire Bus Line
	4850 1950 6850 1950
Wire Bus Line
	6850 1950 6850 900 
Wire Bus Line
	6850 900  4850 900 
Wire Notes Line
	9600 3450 10950 3450
Wire Notes Line
	9600 3450 9600 4350
Wire Notes Line
	9600 4350 9850 4350
Wire Notes Line
	9850 4350 9850 3850
Wire Notes Line
	9850 3850 10950 3850
Wire Notes Line
	10950 3850 10950 3450
Connection ~ 6200 4900
Wire Wire Line
	6150 4900 6200 4900
Wire Wire Line
	6700 5000 6650 5000
Wire Wire Line
	7200 4800 7250 4800
Wire Wire Line
	7250 4800 7250 4550
Wire Wire Line
	7250 4550 6900 4550
Connection ~ 7950 3050
Wire Wire Line
	7950 3200 7950 3050
Wire Wire Line
	3900 3100 3900 3050
Wire Wire Line
	9750 4450 8850 4450
Wire Wire Line
	9200 2450 9100 2450
Wire Wire Line
	9200 2150 9100 2150
Wire Wire Line
	7550 2600 7550 4250
Wire Wire Line
	8250 3100 8250 2250
Wire Wire Line
	8150 3000 8150 1950
Connection ~ 7550 3650
Wire Wire Line
	7550 3650 7650 3650
Wire Wire Line
	6900 3550 7350 3550
Wire Wire Line
	7750 3200 7750 3100
Wire Wire Line
	7350 3150 8600 3150
Wire Wire Line
	8100 2900 8850 2900
Wire Wire Line
	8850 2900 8850 2850
Wire Wire Line
	8150 3000 7300 3000
Wire Wire Line
	8250 3100 7450 3100
Wire Wire Line
	8250 2250 8300 2250
Wire Wire Line
	9050 2550 9200 2550
Wire Wire Line
	9200 2350 8950 2350
Connection ~ 8950 1250
Wire Wire Line
	8950 1300 8950 1250
Connection ~ 8800 2050
Wire Wire Line
	8800 1800 8800 2050
Wire Wire Line
	9150 1250 9150 2250
Connection ~ 8650 1250
Wire Wire Line
	8650 1250 8650 1300
Connection ~ 8250 1250
Wire Wire Line
	8250 1250 8250 1300
Wire Wire Line
	9150 1250 8000 1250
Wire Wire Line
	8700 2050 9200 2050
Wire Wire Line
	8150 1950 9200 1950
Connection ~ 7800 1600
Wire Wire Line
	7800 1550 7800 1650
Wire Wire Line
	7550 1050 7400 1050
Wire Wire Line
	7400 1050 7400 1300
Wire Wire Line
	7300 3000 7300 4250
Wire Wire Line
	7400 1700 7400 1750
Connection ~ 7350 3550
Wire Wire Line
	8850 4150 9300 4150
Wire Wire Line
	8850 3950 9400 3950
Wire Wire Line
	6900 4450 7050 4450
Wire Wire Line
	2500 2550 2500 2650
Wire Wire Line
	2500 5050 2500 5150
Wire Wire Line
	6650 1650 6650 1850
Wire Wire Line
	6100 1850 5650 1850
Wire Wire Line
	9700 4500 9700 4400
Connection ~ 10600 3950
Wire Wire Line
	10600 3950 10600 3900
Wire Wire Line
	10600 3400 10600 3300
Wire Wire Line
	1300 3400 1300 3500
Wire Wire Line
	1700 2400 1300 2400
Wire Wire Line
	3150 3400 3150 2400
Wire Wire Line
	3150 3400 2100 3400
Wire Wire Line
	2100 3400 2100 3150
Connection ~ 3150 3050
Connection ~ 1900 2950
Wire Wire Line
	1900 3050 1900 2950
Wire Wire Line
	1350 2950 1250 2950
Wire Wire Line
	2500 3450 2500 3500
Wire Wire Line
	2100 2950 1850 2950
Connection ~ 1300 2950
Wire Wire Line
	1900 3450 1900 3500
Connection ~ 2500 2600
Wire Wire Line
	3150 2400 2100 2400
Wire Wire Line
	1700 2600 1300 2600
Connection ~ 1300 2600
Wire Wire Line
	1300 2400 1300 3000
Wire Wire Line
	1600 3550 1600 3500
Connection ~ 1600 3500
Wire Wire Line
	1900 3500 1300 3500
Wire Wire Line
	1900 2250 1300 2250
Connection ~ 1600 2250
Wire Wire Line
	1600 2300 1600 2250
Wire Wire Line
	3500 1250 3550 1250
Wire Wire Line
	1300 1750 1300 1150
Connection ~ 1300 1350
Wire Wire Line
	1700 1350 1300 1350
Wire Wire Line
	2100 1150 3150 1150
Connection ~ 10200 5000
Wire Wire Line
	10800 5000 9700 5000
Wire Wire Line
	10800 5000 10800 4650
Connection ~ 8250 3400
Wire Wire Line
	8250 3400 8300 3400
Connection ~ 8750 3650
Wire Wire Line
	8750 3700 8750 3400
Wire Wire Line
	8750 3400 8700 3400
Wire Wire Line
	8250 4950 8900 4950
Wire Wire Line
	8900 4950 8900 4650
Connection ~ 9700 4450
Wire Wire Line
	9700 5000 9700 4900
Wire Wire Line
	3750 5400 3550 5400
Wire Wire Line
	3550 5400 3550 4300
Wire Wire Line
	3750 5100 3650 5100
Wire Wire Line
	9350 5400 9350 4050
Wire Wire Line
	9350 5400 4250 5400
Wire Wire Line
	4250 5100 9250 5100
Wire Wire Line
	3950 1250 4000 1250
Wire Wire Line
	3900 3050 3950 3050
Connection ~ 2500 1350
Connection ~ 5300 5850
Wire Wire Line
	5350 5850 5000 5850
Connection ~ 6450 5850
Wire Wire Line
	6500 5850 6150 5850
Connection ~ 6200 5850
Wire Wire Line
	6450 5900 6450 5850
Wire Wire Line
	5300 5900 5300 5850
Connection ~ 5750 6400
Wire Wire Line
	6450 6300 6450 6400
Wire Wire Line
	6450 6400 5050 6400
Wire Wire Line
	5050 6400 5050 6300
Wire Wire Line
	2500 1400 2500 1300
Wire Wire Line
	1900 2250 1900 2200
Connection ~ 1300 1700
Wire Wire Line
	2100 1700 1850 1700
Wire Wire Line
	2500 2200 2500 2250
Wire Wire Line
	1350 1700 1250 1700
Wire Wire Line
	1900 1800 1900 1700
Connection ~ 1900 1700
Connection ~ 3150 1800
Wire Wire Line
	2100 1900 2100 2150
Wire Wire Line
	2100 2150 3150 2150
Wire Wire Line
	3150 2150 3150 1150
Connection ~ 5750 2700
Wire Wire Line
	5800 2700 5450 2700
Connection ~ 7450 3750
Wire Wire Line
	6900 3750 7450 3750
Wire Wire Line
	7650 4650 7350 4650
Wire Wire Line
	7350 4650 7350 3150
Wire Wire Line
	7650 4450 7450 4450
Wire Wire Line
	7450 4450 7450 3100
Wire Wire Line
	8700 3650 8750 3650
Wire Wire Line
	3850 4000 3850 4050
Connection ~ 4550 3550
Wire Wire Line
	4550 3600 4550 3550
Wire Wire Line
	4500 3550 4600 3550
Wire Wire Line
	4600 3350 3850 3350
Wire Wire Line
	3900 3550 3850 3550
Wire Wire Line
	3850 3350 3850 3600
Connection ~ 3850 3550
Wire Wire Line
	4550 4000 4550 4050
Connection ~ 8250 3650
Wire Wire Line
	7400 4550 7400 3050
Wire Wire Line
	7400 4550 7650 4550
Wire Wire Line
	8900 4650 8850 4650
Wire Wire Line
	6200 2700 6250 2700
Wire Wire Line
	6250 2700 6250 2750
Wire Wire Line
	4950 2700 4550 2700
Wire Wire Line
	5750 4900 5750 4850
Wire Wire Line
	3100 1800 3650 1800
Wire Wire Line
	3650 1800 3650 5100
Wire Wire Line
	9250 5100 9250 4250
Wire Wire Line
	5750 2650 5750 2750
Wire Wire Line
	6200 6400 6200 6300
Connection ~ 6200 6400
Wire Wire Line
	5300 6300 5300 6400
Connection ~ 5300 6400
Wire Wire Line
	6200 5850 6200 5900
Wire Wire Line
	5050 5850 5050 5900
Connection ~ 5050 5850
Wire Wire Line
	5750 6450 5750 6350
Wire Wire Line
	2950 1350 3000 1350
Wire Wire Line
	3000 1350 3000 1400
Wire Wire Line
	4000 1450 3950 1450
Wire Wire Line
	3950 2150 3900 2150
Wire Wire Line
	4250 5250 9300 5250
Wire Wire Line
	9300 5250 9300 4150
Wire Wire Line
	4250 5550 9400 5550
Wire Wire Line
	9400 5550 9400 3950
Wire Wire Line
	3600 5250 3600 3050
Wire Wire Line
	3600 5250 3750 5250
Wire Wire Line
	10300 4000 10300 3950
Wire Wire Line
	10100 4000 10100 3950
Wire Wire Line
	8250 4900 8250 5000
Connection ~ 8250 4950
Wire Wire Line
	8250 3700 8250 3350
Wire Wire Line
	10100 3950 10800 3950
Connection ~ 10300 3950
Wire Wire Line
	10200 4950 10200 5050
Wire Wire Line
	1300 1150 1700 1150
Wire Wire Line
	2550 1350 2100 1350
Wire Wire Line
	1300 2250 1300 2150
Wire Wire Line
	10800 3300 10800 3400
Wire Wire Line
	2100 2600 2500 2600
Wire Wire Line
	2100 3850 2500 3850
Wire Wire Line
	1900 4750 1300 4750
Connection ~ 1600 4750
Wire Wire Line
	1600 4800 1600 4750
Wire Wire Line
	1300 4250 1300 3650
Connection ~ 1300 3850
Wire Wire Line
	1700 3850 1300 3850
Wire Wire Line
	2100 3650 3150 3650
Connection ~ 2500 3850
Wire Wire Line
	1900 4750 1900 4700
Connection ~ 1300 4200
Wire Wire Line
	2100 4200 1850 4200
Wire Wire Line
	2500 4700 2500 4750
Wire Wire Line
	1350 4200 1250 4200
Wire Wire Line
	1900 4300 1900 4200
Connection ~ 1900 4200
Connection ~ 3150 4300
Wire Wire Line
	2100 4400 2100 4650
Wire Wire Line
	2100 4650 3150 4650
Wire Wire Line
	3150 4650 3150 3650
Wire Wire Line
	1300 3650 1700 3650
Wire Wire Line
	1300 4750 1300 4650
Wire Wire Line
	1300 5900 1300 6000
Wire Wire Line
	1700 4900 1300 4900
Wire Wire Line
	3150 5900 3150 4900
Wire Wire Line
	3150 5900 2100 5900
Wire Wire Line
	2100 5900 2100 5650
Connection ~ 3150 5550
Connection ~ 1900 5450
Wire Wire Line
	1900 5550 1900 5450
Wire Wire Line
	1350 5450 1250 5450
Wire Wire Line
	2500 5950 2500 6000
Wire Wire Line
	2100 5450 1850 5450
Connection ~ 1300 5450
Wire Wire Line
	1900 5950 1900 6000
Connection ~ 2500 5100
Wire Wire Line
	3150 4900 2100 4900
Wire Wire Line
	1700 5100 1300 5100
Connection ~ 1300 5100
Wire Wire Line
	1300 4900 1300 5500
Wire Wire Line
	1600 6050 1600 6000
Connection ~ 1600 6000
Wire Wire Line
	1900 6000 1300 6000
Wire Wire Line
	2100 5100 2500 5100
Wire Wire Line
	3550 4300 3100 4300
Wire Wire Line
	3750 5550 3100 5550
Wire Wire Line
	3600 3050 3100 3050
Wire Wire Line
	10800 3900 10800 4250
Connection ~ 10800 3950
Wire Wire Line
	10600 3350 9700 3350
Wire Wire Line
	9700 3350 9700 3900
Connection ~ 10600 3350
Wire Wire Line
	6900 3250 7250 3250
Wire Wire Line
	6050 1650 6050 1850
Connection ~ 6050 1850
Wire Wire Line
	5400 1450 5450 1450
Wire Wire Line
	5650 1050 6650 1050
Wire Wire Line
	6650 1050 6650 1250
Connection ~ 6050 1050
Wire Wire Line
	2500 3800 2500 3900
Wire Wire Line
	6600 1850 6700 1850
Connection ~ 6650 1850
Wire Wire Line
	7300 4250 6900 4250
Wire Wire Line
	8150 3650 8300 3650
Wire Wire Line
	6050 1000 6050 1250
Wire Wire Line
	9350 4050 8850 4050
Wire Wire Line
	9250 4250 8850 4250
Wire Wire Line
	6900 3650 7400 3650
Connection ~ 7400 3650
Connection ~ 7400 1250
Wire Wire Line
	7350 1250 7600 1250
Wire Wire Line
	7800 1600 8100 1600
Wire Wire Line
	8250 1700 8250 1750
Wire Wire Line
	8650 1800 8650 1950
Connection ~ 8650 1950
Wire Wire Line
	9150 2250 9200 2250
Wire Wire Line
	8800 1250 8800 1300
Connection ~ 8800 1250
Wire Wire Line
	8700 2250 8950 2250
Connection ~ 8950 2250
Wire Wire Line
	8950 2350 8950 1800
Wire Wire Line
	8650 2550 8600 2550
Wire Wire Line
	8600 2550 8600 3150
Wire Wire Line
	7400 3050 8200 3050
Wire Wire Line
	8300 2050 8200 2050
Wire Wire Line
	8500 2550 8500 2600
Wire Wire Line
	8500 2600 8100 2600
Wire Wire Line
	7550 2600 7600 2600
Wire Wire Line
	7550 2900 7600 2900
Connection ~ 7550 4150
Connection ~ 7550 2900
Wire Wire Line
	6900 4150 7550 4150
Wire Wire Line
	7550 4250 7650 4250
Wire Wire Line
	8200 2050 8200 3050
Wire Wire Line
	7250 3250 7250 2450
Wire Wire Line
	7250 2450 7800 2450
Wire Wire Line
	7800 2450 7800 2150
Wire Wire Line
	9100 2150 9100 2850
Connection ~ 9100 2450
Wire Wire Line
	4600 3050 4550 3050
Wire Wire Line
	4550 2700 4550 3100
Connection ~ 4550 3050
Connection ~ 7750 3100
Wire Wire Line
	3900 2150 3900 2200
Wire Wire Line
	3950 1450 3950 1500
Wire Wire Line
	7850 3200 7850 3150
Connection ~ 7850 3150
Wire Wire Line
	8050 1050 8100 1050
Wire Wire Line
	8100 1050 8100 1600
Wire Wire Line
	6900 4350 7300 4350
Wire Wire Line
	7300 4350 7300 5000
Wire Wire Line
	7300 5000 7200 5000
Wire Wire Line
	6650 4800 6700 4800
Wire Wire Line
	6250 5000 6200 5000
Wire Wire Line
	6200 5000 6200 4800
Wire Wire Line
	6200 4800 6250 4800
Wire Notes Line
	11050 5150 9450 5150
Wire Notes Line
	11050 5150 11050 3150
Wire Notes Line
	11050 3150 9450 3150
Wire Notes Line
	9450 3150 9450 5150
Wire Bus Line
	3250 6150 3250 950 
Wire Bus Line
	3250 6150 500  6150
Wire Bus Line
	500  6150 500  950 
Wire Bus Line
	500  950  3250 950 
Wire Bus Line
	9850 2950 9850 900 
Wire Bus Line
	9850 2950 7200 2950
Wire Bus Line
	7200 2950 7200 900 
Wire Bus Line
	7200 900  9850 900 
Wire Notes Line
	7050 5800 10100 5800
Wire Notes Line
	8600 5650 8600 6200
Wire Notes Line
	9100 5650 9100 6200
Text Notes 9400 5750 0    60   ~ 0
U2\n\n-\nLM4132-3.3\nLM4132-2.5\nLM4132-2.048
Text Notes 9150 5750 0    60   ~ 0
R13\n\n-\n0\n-\n-
Text Notes 8900 5750 0    60   ~ 0
R14\n\n-\n-\n0\n0
Text Notes 8650 5750 0    60   ~ 0
R16\n\n1k\n-\n-\n-
Text Notes 7750 5750 0    60   ~ 0
Operating Voltage\n\n6.0V\n6.0V\n5.5V\n5.0V
Text Notes 7100 5750 0    60   ~ 0
Ref. Voltage\n\n3.3V\n3.3V\n2.5V\n2.048V
Text Notes 4850 5750 0    60   ~ 0
3.3V Regulator
Text Notes 9800 1000 2    60   ~ 0
SD-Card
Text Notes 4900 1000 0    60   ~ 0
Charge Pump
$Comp
L +3.3V #PWR02
U 1 1 5129EDF6
P 6150 4900
F 0 "#PWR02" H 6150 4860 30  0001 C CNN
F 1 "+3.3V" H 6150 5010 30  0000 C CNN
	1    6150 4900
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 5129EDE9
P 10600 3300
F 0 "#PWR03" H 10600 3260 30  0001 C CNN
F 1 "+3.3V" H 10600 3410 30  0000 C CNN
	1    10600 3300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 5129EDE2
P 8250 3350
F 0 "#PWR04" H 8250 3310 30  0001 C CNN
F 1 "+3.3V" H 8250 3460 30  0000 C CNN
	1    8250 3350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 5129EDDA
P 7350 1250
F 0 "#PWR05" H 7350 1210 30  0001 C CNN
F 1 "+3.3V" H 7350 1360 30  0000 C CNN
	1    7350 1250
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 5129EDD6
P 6500 5850
F 0 "#PWR06" H 6500 5810 30  0001 C CNN
F 1 "+3.3V" H 6500 5960 30  0000 C CNN
	1    6500 5850
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 5129EDCC
P 5750 2650
F 0 "#PWR07" H 5750 2610 30  0001 C CNN
F 1 "+3.3V" H 5750 2760 30  0000 C CNN
	1    5750 2650
	1    0    0    -1  
$EndComp
$Comp
L -1.5V #PWR08
U 1 1 5129ED82
P 2500 2250
F 0 "#PWR08" H 2500 2200 20  0001 C CNN
F 1 "-1.5V" H 2500 2350 30  0000 C CNN
	1    2500 2250
	-1   0    0    1   
$EndComp
$Comp
L -1.5V #PWR09
U 1 1 5129ED7F
P 2500 3500
F 0 "#PWR09" H 2500 3450 20  0001 C CNN
F 1 "-1.5V" H 2500 3600 30  0000 C CNN
	1    2500 3500
	-1   0    0    1   
$EndComp
$Comp
L -1.5V #PWR010
U 1 1 5129ED7D
P 2500 4750
F 0 "#PWR010" H 2500 4700 20  0001 C CNN
F 1 "-1.5V" H 2500 4850 30  0000 C CNN
	1    2500 4750
	-1   0    0    1   
$EndComp
$Comp
L -1.5V #PWR011
U 1 1 5129ED61
P 2500 6000
F 0 "#PWR011" H 2500 5950 20  0001 C CNN
F 1 "-1.5V" H 2500 6100 30  0000 C CNN
	1    2500 6000
	-1   0    0    1   
$EndComp
$Comp
L -1.5V #PWR012
U 1 1 5129ED57
P 6700 1850
F 0 "#PWR012" H 6700 1800 20  0001 C CNN
F 1 "-1.5V" H 6700 1950 30  0000 C CNN
	1    6700 1850
	0    1    1    0   
$EndComp
Text Notes 550  1050 0    60   ~ 0
Low-Pass Filter
Text Notes 9750 3550 0    60   ~ 0
Solder only one!
Text Notes 9500 3250 0    60   ~ 0
Reference
$Comp
L GND #PWR013
U 1 1 510967D0
P 5750 6450
F 0 "#PWR013" H 5750 6450 30  0001 C CNN
F 1 "GND" H 5750 6380 30  0001 C CNN
	1    5750 6450
	1    0    0    -1  
$EndComp
$Comp
L R R27
U 1 1 510964EC
P 7850 2900
F 0 "R27" V 7930 2900 50  0000 C CNN
F 1 "10k" V 7850 2900 50  0000 C CNN
	1    7850 2900
	0    -1   -1   0   
$EndComp
$Comp
L NPN Q3
U 1 1 510963FA
P 8500 2350
F 0 "Q3" V 8500 2200 50  0000 R CNN
F 1 "NPN" V 8400 2250 50  0000 R CNN
	1    8500 2350
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR014
U 1 1 5109635D
P 8250 1750
F 0 "#PWR014" H 8250 1750 30  0001 C CNN
F 1 "GND" H 8250 1680 30  0001 C CNN
	1    8250 1750
	1    0    0    -1  
$EndComp
$Comp
L CP C19
U 1 1 50C75B0D
P 6200 6100
F 0 "C19" H 6050 6200 50  0000 L CNN
F 1 "10µ" H 6250 6000 50  0000 L CNN
	1    6200 6100
	1    0    0    -1  
$EndComp
$Comp
L TST P7
U 1 1 50C863F3
P 7950 3200
F 0 "P7" H 7950 3500 40  0000 C CNN
F 1 "TST" H 7950 3450 30  0000 C CNN
	1    7950 3200
	-1   0    0    1   
$EndComp
$Comp
L TST P5
U 1 1 50C863D6
P 7750 3200
F 0 "P5" H 7750 3500 40  0000 C CNN
F 1 "TST" H 7750 3450 30  0000 C CNN
	1    7750 3200
	-1   0    0    1   
$EndComp
$Comp
L TST P6
U 1 1 50C863C6
P 7850 3200
F 0 "P6" H 7850 3500 40  0000 C CNN
F 1 "TST" H 7850 3450 30  0000 C CNN
	1    7850 3200
	-1   0    0    1   
$EndComp
$Comp
L TST P4
U 1 1 50C863A0
P 4550 3100
F 0 "P4" H 4650 3250 40  0000 C CNN
F 1 "TST" H 4650 3200 30  0000 C CNN
	1    4550 3100
	-1   0    0    1   
$EndComp
$Comp
L BAT54S D10
U 1 1 50C862B0
P 5650 1450
F 0 "D10" H 5750 1350 40  0000 C CNN
F 1 "BAT54S" H 5650 1550 40  0000 C CNN
	1    5650 1450
	0    1    -1   0   
$EndComp
Text Label 5000 1450 2    60   ~ 0
CP
Text Label 7050 4450 0    60   ~ 0
CP
$Comp
L GND #PWR015
U 1 1 50C8602F
P 6050 1000
F 0 "#PWR015" H 6050 1000 30  0001 C CNN
F 1 "GND" H 6050 930 30  0001 C CNN
	1    6050 1000
	1    0    0    1   
$EndComp
$Comp
L CP C21
U 1 1 50C85C28
P 6650 1450
F 0 "C21" H 6700 1550 50  0000 L CNN
F 1 "47µ" H 6700 1350 50  0000 L CNN
	1    6650 1450
	-1   0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 50C85C25
P 6350 1850
F 0 "R17" V 6430 1850 50  0000 C CNN
F 1 "100" V 6350 1850 50  0000 C CNN
	1    6350 1850
	0    1    -1   0   
$EndComp
$Comp
L CP C18
U 1 1 50C85C0E
P 6050 1450
F 0 "C18" H 6100 1550 50  0000 L CNN
F 1 "10µ" H 6100 1350 50  0000 L CNN
	1    6050 1450
	-1   0    0    -1  
$EndComp
$Comp
L CP C16
U 1 1 50C85BEF
P 5200 1450
F 0 "C16" H 5250 1550 50  0000 L CNN
F 1 "1µ" H 5250 1350 50  0000 L CNN
	1    5200 1450
	0    -1   1    0   
$EndComp
$Comp
L CONN_7 P3
U 1 1 50C85B75
P 4300 2750
F 0 "P3" V 4270 2750 60  0000 C CNN
F 1 "CONN_7" V 4370 2750 60  0000 C CNN
	1    4300 2750
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 50C85A60
P 9700 4150
F 0 "R16" V 9780 4150 50  0000 C CNN
F 1 "R" V 9700 4150 50  0000 C CNN
	1    9700 4150
	1    0    0    1   
$EndComp
$Comp
L R R13
U 1 1 50C85A21
P 10800 3650
F 0 "R13" V 10880 3650 50  0000 C CNN
F 1 "R" V 10800 3650 50  0000 C CNN
	1    10800 3650
	1    0    0    1   
$EndComp
$Comp
L GND #PWR016
U 1 1 50C85A08
P 1600 6050
F 0 "#PWR016" H 1600 6050 30  0001 C CNN
F 1 "GND" H 1600 5980 30  0001 C CNN
	1    1600 6050
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 50C85A07
P 1900 4900
F 0 "C7" H 1950 5000 50  0000 L CNN
F 1 "1n" H 1950 4800 50  0000 L CNN
	1    1900 4900
	0    -1   -1   0   
$EndComp
$Comp
L C C8
U 1 1 50C85A06
P 1900 5750
F 0 "C8" H 1950 5850 50  0000 L CNN
F 1 "1n" H 1950 5650 50  0000 L CNN
	1    1900 5750
	-1   0    0    1   
$EndComp
$Comp
L R R4
U 1 1 50C85A05
P 1000 5450
F 0 "R4" V 1080 5450 50  0000 C CNN
F 1 "10k" V 1000 5450 50  0000 C CNN
	1    1000 5450
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 50C85A04
P 1600 5450
F 0 "R8" V 1680 5450 50  0000 C CNN
F 1 "10k" V 1600 5450 50  0000 C CNN
	1    1600 5450
	0    -1   -1   0   
$EndComp
$Comp
L TL074 U1
U 4 1 50C85A03
P 2600 5550
F 0 "U1" H 2650 5750 60  0000 C CNN
F 1 "LMC6464" H 2750 5350 50  0000 C CNN
	4    2600 5550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR017
U 1 1 50C85A02
P 2500 5050
F 0 "#PWR017" H 2500 5140 20  0001 C CNN
F 1 "+5V" H 2500 5140 30  0000 C CNN
	1    2500 5050
	1    0    0    -1  
$EndComp
$Comp
L DIODE D8
U 1 1 50C85A01
P 1900 5100
F 0 "D8" H 1900 5200 40  0000 C CNN
F 1 "DIODE" H 1900 5000 40  0000 C CNN
	1    1900 5100
	1    0    0    -1  
$EndComp
$Comp
L DIODE D4
U 1 1 50C85A00
P 1300 5700
F 0 "D4" H 1300 5800 40  0000 C CNN
F 1 "DIODE" H 1300 5600 40  0000 C CNN
	1    1300 5700
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D3
U 1 1 50C859FB
P 1300 4450
F 0 "D3" H 1300 4550 40  0000 C CNN
F 1 "DIODE" H 1300 4350 40  0000 C CNN
	1    1300 4450
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D7
U 1 1 50C859FA
P 1900 3850
F 0 "D7" H 1900 3950 40  0000 C CNN
F 1 "DIODE" H 1900 3750 40  0000 C CNN
	1    1900 3850
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR018
U 1 1 50C859F9
P 2500 3800
F 0 "#PWR018" H 2500 3890 20  0001 C CNN
F 1 "+5V" H 2500 3890 30  0000 C CNN
	1    2500 3800
	1    0    0    -1  
$EndComp
$Comp
L TL074 U1
U 3 1 50C859F8
P 2600 4300
F 0 "U1" H 2650 4500 60  0000 C CNN
F 1 "LMC6464" H 2750 4100 50  0000 C CNN
	3    2600 4300
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 50C859F7
P 1600 4200
F 0 "R7" V 1680 4200 50  0000 C CNN
F 1 "10k" V 1600 4200 50  0000 C CNN
	1    1600 4200
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 50C859F6
P 1000 4200
F 0 "R3" V 1080 4200 50  0000 C CNN
F 1 "10k" V 1000 4200 50  0000 C CNN
	1    1000 4200
	0    -1   -1   0   
$EndComp
$Comp
L C C6
U 1 1 50C859F5
P 1900 4500
F 0 "C6" H 1950 4600 50  0000 L CNN
F 1 "1n" H 1950 4400 50  0000 L CNN
	1    1900 4500
	-1   0    0    1   
$EndComp
$Comp
L C C5
U 1 1 50C859F4
P 1900 3650
F 0 "C5" H 1950 3750 50  0000 L CNN
F 1 "1n" H 1950 3550 50  0000 L CNN
	1    1900 3650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 50C859F3
P 1600 4800
F 0 "#PWR019" H 1600 4800 30  0001 C CNN
F 1 "GND" H 1600 4730 30  0001 C CNN
	1    1600 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 50C859E0
P 1600 3550
F 0 "#PWR020" H 1600 3550 30  0001 C CNN
F 1 "GND" H 1600 3480 30  0001 C CNN
	1    1600 3550
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 50C859DF
P 1900 2400
F 0 "C3" H 1950 2500 50  0000 L CNN
F 1 "1n" H 1950 2300 50  0000 L CNN
	1    1900 2400
	0    -1   -1   0   
$EndComp
$Comp
L C C4
U 1 1 50C859DE
P 1900 3250
F 0 "C4" H 1950 3350 50  0000 L CNN
F 1 "1n" H 1950 3150 50  0000 L CNN
	1    1900 3250
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 50C859DD
P 1000 2950
F 0 "R2" V 1080 2950 50  0000 C CNN
F 1 "10k" V 1000 2950 50  0000 C CNN
	1    1000 2950
	0    -1   -1   0   
$EndComp
$Comp
L R R6
U 1 1 50C859DC
P 1600 2950
F 0 "R6" V 1680 2950 50  0000 C CNN
F 1 "10k" V 1600 2950 50  0000 C CNN
	1    1600 2950
	0    -1   -1   0   
$EndComp
$Comp
L TL074 U1
U 2 1 50C859DB
P 2600 3050
F 0 "U1" H 2650 3250 60  0000 C CNN
F 1 "LMC6464" H 2750 2850 50  0000 C CNN
	2    2600 3050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR021
U 1 1 50C859DA
P 2500 2550
F 0 "#PWR021" H 2500 2640 20  0001 C CNN
F 1 "+5V" H 2500 2640 30  0000 C CNN
	1    2500 2550
	1    0    0    -1  
$EndComp
$Comp
L DIODE D6
U 1 1 50C859D7
P 1900 2600
F 0 "D6" H 1900 2700 40  0000 C CNN
F 1 "DIODE" H 1900 2500 40  0000 C CNN
	1    1900 2600
	1    0    0    -1  
$EndComp
$Comp
L DIODE D2
U 1 1 50C859D6
P 1300 3200
F 0 "D2" H 1300 3300 40  0000 C CNN
F 1 "DIODE" H 1300 3100 40  0000 C CNN
	1    1300 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R14
U 1 1 50C85546
P 10600 3650
F 0 "R14" V 10680 3650 50  0000 C CNN
F 1 "R" V 10600 3650 50  0000 C CNN
	1    10600 3650
	1    0    0    1   
$EndComp
$Comp
L DIODE D9
U 1 1 50C84D22
P 3750 1250
F 0 "D9" H 3750 1350 40  0000 C CNN
F 1 "DIODE" H 3750 1150 40  0000 C CNN
	1    3750 1250
	-1   0    0    1   
$EndComp
$Comp
L DIODE D1
U 1 1 50C83DCB
P 1300 1950
F 0 "D1" H 1300 2050 40  0000 C CNN
F 1 "DIODE" H 1300 1850 40  0000 C CNN
	1    1300 1950
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D5
U 1 1 50C83DBB
P 1900 1350
F 0 "D5" H 1900 1450 40  0000 C CNN
F 1 "DIODE" H 1900 1250 40  0000 C CNN
	1    1900 1350
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 50C782B4
P 10800 4450
F 0 "C12" H 10850 4550 50  0000 L CNN
F 1 "100n" H 10850 4350 50  0000 L CNN
	1    10800 4450
	1    0    0    1   
$EndComp
$Comp
L CP C22
U 1 1 50C7826F
P 8500 3400
F 0 "C22" H 8550 3500 50  0000 L CNN
F 1 "1µ" H 8550 3300 50  0000 L CNN
	1    8500 3400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR022
U 1 1 50C78092
P 10200 5050
F 0 "#PWR022" H 10200 5050 30  0001 C CNN
F 1 "GND" H 10200 4980 30  0001 C CNN
	1    10200 5050
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR023
U 1 1 50C78068
P 10800 3300
F 0 "#PWR023" H 10800 3390 20  0001 C CNN
F 1 "+5V" H 10800 3390 30  0000 C CNN
	1    10800 3300
	-1   0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 50C78059
P 9700 4700
F 0 "C13" H 9750 4800 50  0000 L CNN
F 1 "100n" H 9750 4600 50  0000 L CNN
	1    9700 4700
	-1   0    0    -1  
$EndComp
$Comp
L LM4120 U2
U 1 1 50C78024
P 10200 4450
F 0 "U2" H 10000 4750 60  0000 C CNN
F 1 "LM4120" H 10400 4100 60  0000 C CNN
	1    10200 4450
	-1   0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 50C75E7C
P 4000 5100
F 0 "R9" V 4080 5100 50  0000 C CNN
F 1 "1k" V 4000 5100 50  0000 C CNN
	1    4000 5100
	0    -1   -1   0   
$EndComp
$Comp
L R R10
U 1 1 50C75E78
P 4000 5250
F 0 "R10" V 4080 5250 50  0000 C CNN
F 1 "1k" V 4000 5250 50  0000 C CNN
	1    4000 5250
	0    -1   -1   0   
$EndComp
$Comp
L R R11
U 1 1 50C75E71
P 4000 5400
F 0 "R11" V 4080 5400 50  0000 C CNN
F 1 "1k" V 4000 5400 50  0000 C CNN
	1    4000 5400
	0    -1   -1   0   
$EndComp
$Comp
L R R12
U 1 1 50C75E5D
P 4000 5550
F 0 "R12" V 4080 5550 50  0000 C CNN
F 1 "1k" V 4000 5550 50  0000 C CNN
	1    4000 5550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR024
U 1 1 50C75DDC
P 3900 2200
F 0 "#PWR024" H 3900 2200 30  0001 C CNN
F 1 "GND" H 3900 2130 30  0001 C CNN
	1    3900 2200
	1    0    0    -1  
$EndComp
Text Label 3950 2050 2    60   ~ 0
AIN3
Text Label 3950 1950 2    60   ~ 0
AIN2
Text Label 3950 1850 2    60   ~ 0
AIN1
Text Label 3950 1750 2    60   ~ 0
AIN0
$Comp
L CONN_5 P2
U 1 1 50C75DC3
P 4350 1950
F 0 "P2" V 4300 1950 50  0000 C CNN
F 1 "CONN_5" V 4400 1950 50  0000 C CNN
	1    4350 1950
	1    0    0    -1  
$EndComp
Text Label 750  5450 2    60   ~ 0
AIN3
Text Label 750  4200 2    60   ~ 0
AIN2
Text Label 750  2950 2    60   ~ 0
AIN1
Text Label 750  1700 2    60   ~ 0
AIN0
$Comp
L +5V #PWR025
U 1 1 50C75D54
P 3500 1250
F 0 "#PWR025" H 3500 1340 20  0001 C CNN
F 1 "+5V" H 3500 1340 30  0000 C CNN
	1    3500 1250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR026
U 1 1 50C75D49
P 3950 1500
F 0 "#PWR026" H 3950 1500 30  0001 C CNN
F 1 "GND" H 3950 1430 30  0001 C CNN
	1    3950 1500
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P1
U 1 1 50C75D41
P 4350 1350
F 0 "P1" V 4300 1350 40  0000 C CNN
F 1 "CONN_2" V 4400 1350 40  0000 C CNN
	1    4350 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 50C75D1F
P 3900 3100
F 0 "#PWR027" H 3900 3100 30  0001 C CNN
F 1 "GND" H 3900 3030 30  0001 C CNN
	1    3900 3100
	1    0    0    -1  
$EndComp
Text Label 3950 2850 2    60   ~ 0
TX
Text Label 3950 2950 2    60   ~ 0
RX
Text Label 3950 2750 2    60   ~ 0
PIO3
Text Label 3950 2650 2    60   ~ 0
PIO2
Text Label 3950 2550 2    60   ~ 0
PIO1
Text Label 3950 2450 2    60   ~ 0
PIO0
Text Label 6900 3950 0    60   ~ 0
RX
Text Label 6900 4050 0    60   ~ 0
TX
Text Label 6900 3450 0    60   ~ 0
PIO3
Text Label 6900 3350 0    60   ~ 0
PIO2
Text Label 6900 3150 0    60   ~ 0
PIO1
Text Label 6900 3050 0    60   ~ 0
PIO0
$Comp
L C C9
U 1 1 50C75B61
P 2750 1350
F 0 "C9" H 2800 1450 50  0000 L CNN
F 1 "100n" H 2800 1250 50  0000 L CNN
	1    2750 1350
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR028
U 1 1 50C75B3B
P 5000 5850
F 0 "#PWR028" H 5000 5940 20  0001 C CNN
F 1 "+5V" H 5000 5940 30  0000 C CNN
	1    5000 5850
	0    -1   -1   0   
$EndComp
$Comp
L C C15
U 1 1 50C75B1B
P 5050 6100
F 0 "C15" H 5100 6200 50  0000 L CNN
F 1 "100n" H 5100 6000 50  0000 L CNN
	1    5050 6100
	-1   0    0    1   
$EndComp
$Comp
L C C20
U 1 1 50C75B13
P 6450 6100
F 0 "C20" H 6300 6200 50  0000 L CNN
F 1 "100n" H 6250 6000 50  0000 L CNN
	1    6450 6100
	-1   0    0    1   
$EndComp
$Comp
L CP C17
U 1 1 50C75B08
P 5300 6100
F 0 "C17" H 5350 6200 50  0000 L CNN
F 1 "10µ" H 5350 6000 50  0000 L CNN
	1    5300 6100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR029
U 1 1 50C7593A
P 2500 1300
F 0 "#PWR029" H 2500 1390 20  0001 C CNN
F 1 "+5V" H 2500 1390 30  0000 C CNN
	1    2500 1300
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 50C75873
P 6950 4800
F 0 "R20" V 7030 4800 50  0000 C CNN
F 1 "330" V 6950 4800 50  0000 C CNN
	1    6950 4800
	0    -1   -1   0   
$EndComp
$Comp
L R R19
U 1 1 50C75848
P 6950 5000
F 0 "R19" V 7030 5000 50  0000 C CNN
F 1 "330" V 6950 5000 50  0000 C CNN
	1    6950 5000
	0    -1   -1   0   
$EndComp
$Comp
L LED D12
U 1 1 50C75840
P 6450 4800
F 0 "D12" H 6450 4900 50  0000 C CNN
F 1 "LED" H 6600 4900 50  0000 C CNN
	1    6450 4800
	1    0    0    -1  
$EndComp
$Comp
L LED D11
U 1 1 50C75805
P 6450 5000
F 0 "D11" H 6450 5100 50  0000 C CNN
F 1 "LED" H 6600 5100 50  0000 C CNN
	1    6450 5000
	1    0    0    -1  
$EndComp
$Comp
L TL074 U1
U 1 1 50C7575E
P 2600 1800
F 0 "U1" H 2650 2000 60  0000 C CNN
F 1 "LMC6464" H 2750 1600 50  0000 C CNN
	1    2600 1800
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 50C7575D
P 1600 1700
F 0 "R5" V 1680 1700 50  0000 C CNN
F 1 "10k" V 1600 1700 50  0000 C CNN
	1    1600 1700
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 50C7575C
P 1000 1700
F 0 "R1" V 1080 1700 50  0000 C CNN
F 1 "10k" V 1000 1700 50  0000 C CNN
	1    1000 1700
	0    -1   -1   0   
$EndComp
$Comp
L C C2
U 1 1 50C7575B
P 1900 2000
F 0 "C2" H 1950 2100 50  0000 L CNN
F 1 "1n" H 1950 1900 50  0000 L CNN
	1    1900 2000
	-1   0    0    1   
$EndComp
$Comp
L C C1
U 1 1 50C7575A
P 1900 1150
F 0 "C1" H 1950 1250 50  0000 L CNN
F 1 "1n" H 1950 1050 50  0000 L CNN
	1    1900 1150
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR030
U 1 1 50C75759
P 1600 2300
F 0 "#PWR030" H 1600 2300 30  0001 C CNN
F 1 "GND" H 1600 2230 30  0001 C CNN
	1    1600 2300
	1    0    0    -1  
$EndComp
$Comp
L NPN Q1
U 1 1 50C746EB
P 8850 2650
F 0 "Q1" V 8850 2500 50  0000 R CNN
F 1 "NPN" V 8750 2550 50  0000 R CNN
	1    8850 2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR031
U 1 1 50C7467A
P 8250 5000
F 0 "#PWR031" H 8250 5000 30  0001 C CNN
F 1 "GND" H 8250 4930 30  0001 C CNN
	1    8250 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 50C745FB
P 8750 3700
F 0 "#PWR032" H 8750 3700 30  0001 C CNN
F 1 "GND" H 8750 3630 30  0001 C CNN
	1    8750 3700
	1    0    0    -1  
$EndComp
$Comp
L C C23
U 1 1 50C745E6
P 8500 3650
F 0 "C23" H 8550 3750 50  0000 L CNN
F 1 "100n" H 8550 3550 50  0000 L CNN
	1    8500 3650
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D13
U 1 1 50C74546
P 8500 2050
F 0 "D13" H 8500 2150 40  0000 C CNN
F 1 "DIODE" H 8500 1950 40  0000 C CNN
	1    8500 2050
	-1   0    0    1   
$EndComp
$Comp
L R R22
U 1 1 50C7426A
P 7850 2600
F 0 "R22" V 7930 2600 50  0000 C CNN
F 1 "10k" V 7850 2600 50  0000 C CNN
	1    7850 2600
	0    -1   -1   0   
$EndComp
$Comp
L R R23
U 1 1 50C74266
P 8950 1550
F 0 "R23" V 9030 1550 50  0000 C CNN
F 1 "10k" V 8950 1550 50  0000 C CNN
	1    8950 1550
	1    0    0    -1  
$EndComp
$Comp
L R R25
U 1 1 50C74262
P 8800 1550
F 0 "R25" V 8880 1550 50  0000 C CNN
F 1 "10k" V 8800 1550 50  0000 C CNN
	1    8800 1550
	1    0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 50C741E0
P 7800 1050
F 0 "R24" V 7880 1050 50  0000 C CNN
F 1 "10k" V 7800 1050 50  0000 C CNN
	1    7800 1050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR033
U 1 1 50C740C1
P 7400 1750
F 0 "#PWR033" H 7400 1750 30  0001 C CNN
F 1 "GND" H 7400 1680 30  0001 C CNN
	1    7400 1750
	1    0    0    -1  
$EndComp
$Comp
L R R21
U 1 1 50C74066
P 7800 1900
F 0 "R21" V 7880 1900 50  0000 C CNN
F 1 "1k" V 7800 1900 50  0000 C CNN
	1    7800 1900
	1    0    0    -1  
$EndComp
$Comp
L PNP Q2
U 1 1 50C7404B
P 7800 1350
F 0 "Q2" H 7800 1200 60  0000 R CNN
F 1 "PNP" H 7800 1500 60  0000 R CNN
	1    7800 1350
	0    1    -1   0   
$EndComp
$Comp
L C C24
U 1 1 50C74003
P 8250 1500
F 0 "C24" H 8300 1600 50  0000 L CNN
F 1 "100n" H 8300 1400 50  0000 L CNN
	1    8250 1500
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 50C61CCF
P 7900 3650
F 0 "R18" V 7980 3650 50  0000 C CNN
F 1 "1k" V 7900 3650 50  0000 C CNN
	1    7900 3650
	0    1    1    0   
$EndComp
$Comp
L CP C25
U 1 1 50C61C6E
P 7400 1500
F 0 "C25" H 7450 1600 50  0000 L CNN
F 1 "47µ" H 7450 1400 50  0000 L CNN
	1    7400 1500
	1    0    0    -1  
$EndComp
$Comp
L R R26
U 1 1 50C61B6A
P 8650 1550
F 0 "R26" V 8730 1550 50  0000 C CNN
F 1 "10k" V 8650 1550 50  0000 C CNN
	1    8650 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 50C61B52
P 9100 2850
F 0 "#PWR034" H 9100 2850 30  0001 C CNN
F 1 "GND" H 9100 2780 30  0001 C CNN
	1    9100 2850
	1    0    0    -1  
$EndComp
$Comp
L SD/MMC P8
U 1 1 50C61B28
P 9600 2350
F 0 "P8" H 9450 2850 60  0000 C CNN
F 1 "SD/MMC" H 9600 1850 60  0000 C CNN
	1    9600 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR035
U 1 1 50C61639
P 4550 4050
F 0 "#PWR035" H 4550 4050 30  0001 C CNN
F 1 "GND" H 4550 3980 30  0001 C CNN
	1    4550 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR036
U 1 1 50C61636
P 3850 4050
F 0 "#PWR036" H 3850 4050 30  0001 C CNN
F 1 "GND" H 3850 3980 30  0001 C CNN
	1    3850 4050
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 50C6162E
P 3850 3800
F 0 "C10" H 3700 3900 50  0000 L CNN
F 1 "10p" H 3900 3700 50  0000 L CNN
	1    3850 3800
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 50C6162B
P 4550 3800
F 0 "C11" H 4600 3900 50  0000 L CNN
F 1 "10p" H 4600 3700 50  0000 L CNN
	1    4550 3800
	1    0    0    -1  
$EndComp
$Comp
L CRYSTAL X1
U 1 1 50C61610
P 4200 3550
F 0 "X1" H 4200 3700 60  0000 C CNN
F 1 "7.68MHz" H 4200 3400 60  0000 C CNN
	1    4200 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR037
U 1 1 50C615AC
P 6250 2750
F 0 "#PWR037" H 6250 2750 30  0001 C CNN
F 1 "GND" H 6250 2680 30  0001 C CNN
	1    6250 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR038
U 1 1 50C614A4
P 5750 4900
F 0 "#PWR038" H 5750 4900 30  0001 C CNN
F 1 "GND" H 5750 4830 30  0001 C CNN
	1    5750 4900
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 50C61485
P 5200 2700
F 0 "R15" V 5280 2700 50  0000 C CNN
F 1 "10k" V 5200 2700 50  0000 C CNN
	1    5200 2700
	0    -1   -1   0   
$EndComp
$Comp
L C C14
U 1 1 50C61466
P 6000 2700
F 0 "C14" H 6050 2800 50  0000 L CNN
F 1 "100n" H 6050 2600 50  0000 L CNN
	1    6000 2700
	0    -1   -1   0   
$EndComp
$Comp
L MCP3204 IC2
U 1 1 50C61441
P 8250 4300
F 0 "IC2" H 8600 4750 60  0000 C CNN
F 1 "MCP3204" H 8000 3850 60  0000 C CNN
	1    8250 4300
	-1   0    0    -1  
$EndComp
$Comp
L ATTINY2313-S IC1
U 1 1 50C60EB9
P 5750 3850
F 0 "IC1" H 4900 4800 60  0000 C CNN
F 1 "ATTINY2313-S" H 6350 4800 60  0000 C CNN
F 2 "SO20" H 4950 3000 60  0001 C CNN
	1    5750 3850
	1    0    0    -1  
$EndComp
$EndSCHEMATC
