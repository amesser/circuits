EESchema Schematic File Version 2  date Mo 15 Dez 2014 21:39:26 CET
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
LIBS:amesser-pmic
LIBS:amesser-linear
LIBS:mppt-loader-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date "15 dec 2014"
Rev "1"
Comp "Copyright (c) 2014 Andreas Messer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L IR2110 U201
U 1 1 547AE63C
P 2950 3350
F 0 "U201" H 3200 3850 60  0000 C CNN
F 1 "IR2110" H 3300 2853 60  0000 C CNN
F 2 "" H 2950 3350 60  0000 C CNN
F 3 "" H 2950 3350 60  0000 C CNN
	1    2950 3350
	1    0    0    -1  
$EndComp
$Comp
L MOS_N Q202
U 1 1 547AE678
P 4500 3550
F 0 "Q202" H 4510 3720 60  0000 R CNN
F 1 "IRF 530NS" H 4510 3400 60  0000 R CNN
F 2 "~" H 4500 3550 60  0000 C CNN
F 3 "~" H 4500 3550 60  0000 C CNN
	1    4500 3550
	1    0    0    -1  
$EndComp
$Comp
L MOS_N Q201
U 1 1 547AE685
P 4500 2950
F 0 "Q201" H 4510 3120 60  0000 R CNN
F 1 "IRF 530NS" H 4510 2800 60  0000 R CNN
F 2 "~" H 4500 2950 60  0000 C CNN
F 3 "~" H 4500 2950 60  0000 C CNN
	1    4500 2950
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L201
U 1 1 547AE68D
P 5150 3200
F 0 "L201" H 5150 3300 50  0000 C CNN
F 1 "22u" H 5150 3150 50  0000 C CNN
F 2 "~" H 5150 3200 60  0000 C CNN
F 3 "~" H 5150 3200 60  0000 C CNN
	1    5150 3200
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D207
U 1 1 547AE6EF
P 4800 3600
F 0 "D207" H 4800 3700 40  0000 C CNN
F 1 "DIODESCH" H 4800 3500 40  0000 C CNN
F 2 "~" H 4800 3600 60  0000 C CNN
F 3 "~" H 4800 3600 60  0000 C CNN
	1    4800 3600
	0    -1   -1   0   
$EndComp
$Comp
L R R202
U 1 1 547AE774
P 3950 3550
F 0 "R202" V 4030 3550 40  0000 C CNN
F 1 "10" V 3957 3551 40  0000 C CNN
F 2 "~" V 3880 3550 30  0000 C CNN
F 3 "~" H 3950 3550 30  0000 C CNN
	1    3950 3550
	0    -1   -1   0   
$EndComp
$Comp
L R R201
U 1 1 547AE781
P 3950 2950
F 0 "R201" V 4030 2950 40  0000 C CNN
F 1 "10" V 3957 2951 40  0000 C CNN
F 2 "~" V 3880 2950 30  0000 C CNN
F 3 "~" H 3950 2950 30  0000 C CNN
	1    3950 2950
	0    -1   -1   0   
$EndComp
$Comp
L R R207
U 1 1 547AEABF
P 5750 3200
F 0 "R207" V 5830 3200 40  0000 C CNN
F 1 "0R1" V 5757 3201 40  0000 C CNN
F 2 "~" V 5680 3200 30  0000 C CNN
F 3 "~" H 5750 3200 30  0000 C CNN
	1    5750 3200
	0    -1   -1   0   
$EndComp
$Comp
L CP C222
U 1 1 547AEC25
P 6450 3600
F 0 "C222" H 6500 3700 40  0000 L CNN
F 1 "100u" H 6500 3500 40  0000 L CNN
F 2 "~" H 6550 3450 30  0000 C CNN
F 3 "~" H 6450 3600 300 0000 C CNN
	1    6450 3600
	1    0    0    -1  
$EndComp
$Comp
L CP C224
U 1 1 547AEC3D
P 6650 3600
F 0 "C224" H 6700 3700 40  0000 L CNN
F 1 "100u" H 6700 3500 40  0000 L CNN
F 2 "~" H 6750 3450 30  0000 C CNN
F 3 "~" H 6650 3600 300 0000 C CNN
	1    6650 3600
	1    0    0    -1  
$EndComp
$Comp
L C C218
U 1 1 547AEC73
P 6050 3600
F 0 "C218" H 6050 3700 40  0000 L CNN
F 1 "100n" H 6056 3515 40  0000 L CNN
F 2 "~" H 6088 3450 30  0000 C CNN
F 3 "~" H 6050 3600 60  0000 C CNN
	1    6050 3600
	1    0    0    -1  
$EndComp
$Comp
L C C220
U 1 1 547AECE0
P 6250 3600
F 0 "C220" H 6250 3700 40  0000 L CNN
F 1 "100n" H 6256 3515 40  0000 L CNN
F 2 "~" H 6288 3450 30  0000 C CNN
F 3 "~" H 6250 3600 60  0000 C CNN
	1    6250 3600
	1    0    0    -1  
$EndComp
$Comp
L CP C214
U 1 1 547AED3D
P 5150 2700
F 0 "C214" H 5200 2800 40  0000 L CNN
F 1 "100u" H 5200 2600 40  0000 L CNN
F 2 "~" H 5250 2550 30  0000 C CNN
F 3 "~" H 5150 2700 300 0000 C CNN
	1    5150 2700
	1    0    0    -1  
$EndComp
$Comp
L CP C216
U 1 1 547AED43
P 5350 2700
F 0 "C216" H 5400 2800 40  0000 L CNN
F 1 "100u" H 5400 2600 40  0000 L CNN
F 2 "~" H 5450 2550 30  0000 C CNN
F 3 "~" H 5350 2700 300 0000 C CNN
	1    5350 2700
	1    0    0    -1  
$EndComp
$Comp
L C C209
U 1 1 547AED49
P 4750 2700
F 0 "C209" H 4750 2800 40  0000 L CNN
F 1 "100n" H 4756 2615 40  0000 L CNN
F 2 "~" H 4788 2550 30  0000 C CNN
F 3 "~" H 4750 2700 60  0000 C CNN
	1    4750 2700
	1    0    0    -1  
$EndComp
$Comp
L C C211
U 1 1 547AED4F
P 4950 2700
F 0 "C211" H 4950 2800 40  0000 L CNN
F 1 "100n" H 4956 2615 40  0000 L CNN
F 2 "~" H 4988 2550 30  0000 C CNN
F 3 "~" H 4950 2700 60  0000 C CNN
	1    4950 2700
	1    0    0    -1  
$EndComp
$Comp
L CP C207
U 1 1 547AEFD7
P 3950 2650
F 0 "C207" H 4000 2750 40  0000 L CNN
F 1 "10u" H 4000 2550 40  0000 L CNN
F 2 "~" H 4050 2500 30  0000 C CNN
F 3 "~" H 3950 2650 300 0000 C CNN
	1    3950 2650
	0    -1   -1   0   
$EndComp
$Comp
L CP C203
U 1 1 547AF195
P 3350 2150
F 0 "C203" H 3400 2250 40  0000 L CNN
F 1 "10u" H 3400 2050 40  0000 L CNN
F 2 "~" H 3450 2000 30  0000 C CNN
F 3 "~" H 3350 2150 300 0000 C CNN
	1    3350 2150
	0    -1   -1   0   
$EndComp
$Comp
L C C204
U 1 1 547AF1C0
P 3350 2350
F 0 "C204" H 3350 2450 40  0000 L CNN
F 1 "100n" H 3356 2265 40  0000 L CNN
F 2 "~" H 3388 2200 30  0000 C CNN
F 3 "~" H 3350 2350 60  0000 C CNN
	1    3350 2350
	0    -1   -1   0   
$EndComp
Text HLabel 4600 2400 1    60   Input ~ 0
VINA
$Comp
L C C201
U 1 1 547B00FE
P 2550 2650
F 0 "C201" H 2550 2750 40  0000 L CNN
F 1 "100n" H 2556 2565 40  0000 L CNN
F 2 "~" H 2588 2500 30  0000 C CNN
F 3 "~" H 2550 2650 60  0000 C CNN
	1    2550 2650
	0    -1   -1   0   
$EndComp
Text HLabel 1650 3100 0    60   Input ~ 0
HIGHSIDEA
Text HLabel 1650 3550 0    60   Input ~ 0
LOWSIDEA
Text HLabel 6750 3200 2    60   Output ~ 0
VOUTA
Text HLabel 6250 3100 2    60   Output ~ 0
SENSEA_VOUT
Text HLabel 6250 3000 2    60   Output ~ 0
SENSEA_I
$Comp
L IR2110 U202
U 1 1 547B0CAA
P 2950 6350
F 0 "U202" H 3200 6850 60  0000 C CNN
F 1 "IR2110" H 3300 5853 60  0000 C CNN
F 2 "" H 2950 6350 60  0000 C CNN
F 3 "" H 2950 6350 60  0000 C CNN
	1    2950 6350
	1    0    0    -1  
$EndComp
$Comp
L MOS_N Q204
U 1 1 547B0CB0
P 4500 6550
F 0 "Q204" H 4510 6720 60  0000 R CNN
F 1 "IRF 530NS" H 4510 6400 60  0000 R CNN
F 2 "~" H 4500 6550 60  0000 C CNN
F 3 "~" H 4500 6550 60  0000 C CNN
	1    4500 6550
	1    0    0    -1  
$EndComp
$Comp
L MOS_N Q203
U 1 1 547B0CB6
P 4500 5950
F 0 "Q203" H 4510 6120 60  0000 R CNN
F 1 "IRF 530NS" H 4510 5800 60  0000 R CNN
F 2 "~" H 4500 5950 60  0000 C CNN
F 3 "~" H 4500 5950 60  0000 C CNN
	1    4500 5950
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L202
U 1 1 547B0CBC
P 5150 6200
F 0 "L202" H 5150 6300 50  0000 C CNN
F 1 "22u" H 5150 6150 50  0000 C CNN
F 2 "~" H 5150 6200 60  0000 C CNN
F 3 "~" H 5150 6200 60  0000 C CNN
	1    5150 6200
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D208
U 1 1 547B0CC2
P 4800 6600
F 0 "D208" H 4800 6700 40  0000 C CNN
F 1 "DIODESCH" H 4800 6500 40  0000 C CNN
F 2 "~" H 4800 6600 60  0000 C CNN
F 3 "~" H 4800 6600 60  0000 C CNN
	1    4800 6600
	0    -1   -1   0   
$EndComp
$Comp
L R R204
U 1 1 547B0CCE
P 3950 6550
F 0 "R204" V 4030 6550 40  0000 C CNN
F 1 "10" V 3957 6551 40  0000 C CNN
F 2 "~" V 3880 6550 30  0000 C CNN
F 3 "~" H 3950 6550 30  0000 C CNN
	1    3950 6550
	0    -1   -1   0   
$EndComp
$Comp
L R R203
U 1 1 547B0CD4
P 3950 5950
F 0 "R203" V 4030 5950 40  0000 C CNN
F 1 "10" V 3957 5951 40  0000 C CNN
F 2 "~" V 3880 5950 30  0000 C CNN
F 3 "~" H 3950 5950 30  0000 C CNN
	1    3950 5950
	0    -1   -1   0   
$EndComp
$Comp
L R R208
U 1 1 547B0CDA
P 5750 6200
F 0 "R208" V 5830 6200 40  0000 C CNN
F 1 "0R1" V 5757 6201 40  0000 C CNN
F 2 "~" V 5680 6200 30  0000 C CNN
F 3 "~" H 5750 6200 30  0000 C CNN
	1    5750 6200
	0    -1   -1   0   
$EndComp
$Comp
L CP C223
U 1 1 547B0CE0
P 6450 6600
F 0 "C223" H 6500 6700 40  0000 L CNN
F 1 "100u" H 6500 6500 40  0000 L CNN
F 2 "~" H 6550 6450 30  0000 C CNN
F 3 "~" H 6450 6600 300 0000 C CNN
	1    6450 6600
	1    0    0    -1  
$EndComp
$Comp
L CP C225
U 1 1 547B0CE6
P 6650 6600
F 0 "C225" H 6700 6700 40  0000 L CNN
F 1 "100u" H 6700 6500 40  0000 L CNN
F 2 "~" H 6750 6450 30  0000 C CNN
F 3 "~" H 6650 6600 300 0000 C CNN
	1    6650 6600
	1    0    0    -1  
$EndComp
$Comp
L C C219
U 1 1 547B0CEC
P 6050 6600
F 0 "C219" H 6050 6700 40  0000 L CNN
F 1 "100n" H 6056 6515 40  0000 L CNN
F 2 "~" H 6088 6450 30  0000 C CNN
F 3 "~" H 6050 6600 60  0000 C CNN
	1    6050 6600
	1    0    0    -1  
$EndComp
$Comp
L C C221
U 1 1 547B0CF2
P 6250 6600
F 0 "C221" H 6250 6700 40  0000 L CNN
F 1 "100n" H 6256 6515 40  0000 L CNN
F 2 "~" H 6288 6450 30  0000 C CNN
F 3 "~" H 6250 6600 60  0000 C CNN
	1    6250 6600
	1    0    0    -1  
$EndComp
$Comp
L CP C215
U 1 1 547B0CF8
P 5150 5700
F 0 "C215" H 5200 5800 40  0000 L CNN
F 1 "100u" H 5200 5600 40  0000 L CNN
F 2 "~" H 5250 5550 30  0000 C CNN
F 3 "~" H 5150 5700 300 0000 C CNN
	1    5150 5700
	1    0    0    -1  
$EndComp
$Comp
L CP C217
U 1 1 547B0CFE
P 5350 5700
F 0 "C217" H 5400 5800 40  0000 L CNN
F 1 "100u" H 5400 5600 40  0000 L CNN
F 2 "~" H 5450 5550 30  0000 C CNN
F 3 "~" H 5350 5700 300 0000 C CNN
	1    5350 5700
	1    0    0    -1  
$EndComp
$Comp
L C C210
U 1 1 547B0D04
P 4750 5700
F 0 "C210" H 4750 5800 40  0000 L CNN
F 1 "100n" H 4756 5615 40  0000 L CNN
F 2 "~" H 4788 5550 30  0000 C CNN
F 3 "~" H 4750 5700 60  0000 C CNN
	1    4750 5700
	1    0    0    -1  
$EndComp
$Comp
L C C212
U 1 1 547B0D0A
P 4950 5700
F 0 "C212" H 4950 5800 40  0000 L CNN
F 1 "100n" H 4956 5615 40  0000 L CNN
F 2 "~" H 4988 5550 30  0000 C CNN
F 3 "~" H 4950 5700 60  0000 C CNN
	1    4950 5700
	1    0    0    -1  
$EndComp
$Comp
L CP C208
U 1 1 547B0D1C
P 3950 5650
F 0 "C208" H 4000 5750 40  0000 L CNN
F 1 "10u" H 4000 5550 40  0000 L CNN
F 2 "~" H 4050 5500 30  0000 C CNN
F 3 "~" H 3950 5650 300 0000 C CNN
	1    3950 5650
	0    -1   -1   0   
$EndComp
$Comp
L CP C205
U 1 1 547B0D28
P 3350 5150
F 0 "C205" H 3400 5250 40  0000 L CNN
F 1 "10u" H 3400 5050 40  0000 L CNN
F 2 "~" H 3450 5000 30  0000 C CNN
F 3 "~" H 3350 5150 300 0000 C CNN
	1    3350 5150
	0    -1   -1   0   
$EndComp
$Comp
L C C206
U 1 1 547B0D2E
P 3350 5350
F 0 "C206" H 3350 5450 40  0000 L CNN
F 1 "100n" H 3356 5265 40  0000 L CNN
F 2 "~" H 3388 5200 30  0000 C CNN
F 3 "~" H 3350 5350 60  0000 C CNN
	1    3350 5350
	0    -1   -1   0   
$EndComp
Text HLabel 4600 5400 1    60   Input ~ 0
VINB
$Comp
L C C202
U 1 1 547B0D41
P 2550 5650
F 0 "C202" H 2550 5750 40  0000 L CNN
F 1 "100n" H 2556 5565 40  0000 L CNN
F 2 "~" H 2588 5500 30  0000 C CNN
F 3 "~" H 2550 5650 60  0000 C CNN
	1    2550 5650
	0    -1   -1   0   
$EndComp
Text HLabel 1650 6100 0    60   Input ~ 0
HIGHSIDEB
Text HLabel 1650 6550 0    60   Input ~ 0
LOWSIDEB
Text HLabel 6750 6200 2    60   Output ~ 0
VOUTB
Text HLabel 6250 6100 2    60   Output ~ 0
SENSEB_VOUT
Text HLabel 6250 6000 2    60   Output ~ 0
SENSEB_I
$Comp
L LM393 U203
U 1 1 547B11CE
P 5300 1900
F 0 "U203" H 5450 2050 60  0000 C CNN
F 1 "LM393" H 5500 1700 60  0000 C CNN
F 2 "" H 5300 1900 60  0000 C CNN
F 3 "" H 5300 1900 60  0000 C CNN
	1    5300 1900
	-1   0    0    -1  
$EndComp
$Comp
L R R211
U 1 1 547B1224
P 6050 2500
F 0 "R211" V 6130 2500 40  0000 C CNN
F 1 "10k" V 6057 2501 40  0000 C CNN
F 2 "~" V 5980 2500 30  0000 C CNN
F 3 "~" H 6050 2500 30  0000 C CNN
	1    6050 2500
	1    0    0    -1  
$EndComp
$Comp
L R R209
U 1 1 547B1231
P 5850 2500
F 0 "R209" V 5930 2500 40  0000 C CNN
F 1 "10k" V 5857 2501 40  0000 C CNN
F 2 "~" V 5780 2500 30  0000 C CNN
F 3 "~" H 5850 2500 30  0000 C CNN
	1    5850 2500
	1    0    0    -1  
$EndComp
$Comp
L R R205
U 1 1 547B186B
P 4500 1900
F 0 "R205" V 4580 1900 40  0000 C CNN
F 1 "1k" V 4507 1901 40  0000 C CNN
F 2 "~" V 4430 1900 30  0000 C CNN
F 3 "~" H 4500 1900 30  0000 C CNN
	1    4500 1900
	0    -1   -1   0   
$EndComp
$Comp
L LM393 U203
U 2 1 547B1C25
P 5300 4900
F 0 "U203" H 5450 5050 60  0000 C CNN
F 1 "LM393" H 5500 4700 60  0000 C CNN
F 2 "" H 5300 4900 60  0000 C CNN
F 3 "" H 5300 4900 60  0000 C CNN
	2    5300 4900
	-1   0    0    -1  
$EndComp
$Comp
L R R206
U 1 1 547B1C36
P 4500 4900
F 0 "R206" V 4580 4900 40  0000 C CNN
F 1 "1k" V 4507 4901 40  0000 C CNN
F 2 "~" V 4430 4900 30  0000 C CNN
F 3 "~" H 4500 4900 30  0000 C CNN
	1    4500 4900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4600 3150 4600 3350
Connection ~ 4600 3200
Wire Wire Line
	4600 3750 4600 4000
Wire Wire Line
	4800 3200 4800 3400
Wire Wire Line
	4200 3550 4300 3550
Connection ~ 4800 3200
Wire Wire Line
	5400 3200 5500 3200
Wire Wire Line
	6000 3200 6750 3200
Wire Wire Line
	6050 2750 6050 3400
Wire Wire Line
	6250 3200 6250 3400
Connection ~ 6050 3200
Wire Wire Line
	6450 3200 6450 3400
Connection ~ 6250 3200
Wire Wire Line
	6650 3200 6650 3400
Connection ~ 6450 3200
Wire Wire Line
	4600 2400 4600 2750
Wire Wire Line
	4600 2450 5350 2450
Wire Wire Line
	4750 2450 4750 2500
Wire Wire Line
	4950 2450 4950 2500
Connection ~ 4750 2450
Wire Wire Line
	5150 2450 5150 2500
Connection ~ 4950 2450
Wire Wire Line
	5350 2450 5350 2500
Connection ~ 5150 2450
Wire Wire Line
	4750 2900 4750 3050
Wire Wire Line
	4750 2950 5350 2950
Wire Wire Line
	5350 2950 5350 2900
Wire Wire Line
	4950 2900 4950 2950
Connection ~ 4950 2950
Wire Wire Line
	5150 2900 5150 2950
Connection ~ 5150 2950
Wire Wire Line
	3550 3550 3700 3550
Wire Wire Line
	3550 3200 4900 3200
Wire Wire Line
	3700 2950 3650 2950
Wire Wire Line
	3650 3100 3550 3100
Wire Wire Line
	4200 2950 4300 2950
Wire Wire Line
	3650 2950 3650 3100
Connection ~ 4250 3200
Wire Wire Line
	4150 2650 4250 2650
Wire Wire Line
	4250 2650 4250 3200
Wire Wire Line
	3550 3000 3600 3000
Wire Wire Line
	3600 3000 3600 2650
Wire Wire Line
	3550 2650 3750 2650
Connection ~ 3600 2650
Wire Wire Line
	3100 1100 3100 2700
Wire Wire Line
	3100 2650 3150 2650
Wire Wire Line
	3550 2150 3600 2150
Wire Wire Line
	3550 2350 3700 2350
Connection ~ 3600 2350
Wire Wire Line
	3100 2350 3150 2350
Connection ~ 3100 2650
Wire Wire Line
	3100 2150 3150 2150
Connection ~ 3100 2350
Connection ~ 3100 2150
Connection ~ 6650 3200
Connection ~ 4600 2450
Wire Wire Line
	2300 2650 2300 4150
Wire Wire Line
	2300 2650 2350 2650
Wire Wire Line
	2800 2650 2750 2650
Wire Wire Line
	2800 2050 2800 2700
Connection ~ 2800 2650
Wire Wire Line
	2800 4000 2800 3950
Wire Wire Line
	3100 3950 3100 4300
Wire Wire Line
	1650 3100 2350 3100
Wire Wire Line
	2150 3350 2350 3350
Wire Wire Line
	1650 3550 2350 3550
Wire Wire Line
	5450 3000 6250 3000
Wire Wire Line
	5450 3000 5450 3200
Connection ~ 5450 3200
Wire Wire Line
	6250 3100 6050 3100
Wire Wire Line
	4600 6150 4600 6350
Connection ~ 4600 6200
Wire Wire Line
	4600 6750 4600 7000
Wire Wire Line
	4800 6200 4800 6400
Wire Wire Line
	4800 7000 4800 6800
Wire Wire Line
	4200 6550 4300 6550
Connection ~ 4800 6200
Wire Wire Line
	5400 6200 5500 6200
Wire Wire Line
	6000 6200 6750 6200
Wire Wire Line
	6050 5800 6050 6400
Wire Wire Line
	6250 6200 6250 6400
Connection ~ 6050 6200
Wire Wire Line
	6450 6200 6450 6400
Connection ~ 6250 6200
Wire Wire Line
	6650 6200 6650 6400
Connection ~ 6450 6200
Wire Wire Line
	4600 5400 4600 5750
Wire Wire Line
	4600 5450 5350 5450
Wire Wire Line
	4750 5450 4750 5500
Wire Wire Line
	4950 5450 4950 5500
Connection ~ 4750 5450
Wire Wire Line
	5150 5450 5150 5500
Connection ~ 4950 5450
Wire Wire Line
	5350 5450 5350 5500
Connection ~ 5150 5450
Wire Wire Line
	4750 5900 4750 6050
Wire Wire Line
	4750 5950 5350 5950
Wire Wire Line
	5350 5950 5350 5900
Wire Wire Line
	4950 5900 4950 5950
Connection ~ 4950 5950
Wire Wire Line
	5150 5900 5150 5950
Connection ~ 5150 5950
Wire Wire Line
	6050 7000 6050 6800
Wire Wire Line
	3550 6550 3700 6550
Wire Wire Line
	3550 6200 4900 6200
Wire Wire Line
	3700 5950 3650 5950
Wire Wire Line
	3650 6100 3550 6100
Wire Wire Line
	4200 5950 4300 5950
Wire Wire Line
	3650 5950 3650 6100
Connection ~ 4250 6200
Wire Wire Line
	4150 5650 4250 5650
Wire Wire Line
	4250 5650 4250 6200
Wire Wire Line
	3550 6000 3600 6000
Wire Wire Line
	3600 6000 3600 5650
Wire Wire Line
	3550 5650 3750 5650
Connection ~ 3600 5650
Wire Wire Line
	3100 5650 3150 5650
Wire Wire Line
	3550 5150 3600 5150
Wire Wire Line
	3550 5350 3700 5350
Connection ~ 3600 5350
Wire Wire Line
	3100 5350 3150 5350
Connection ~ 3100 5650
Wire Wire Line
	3100 5150 3150 5150
Connection ~ 3100 5350
Connection ~ 3100 5150
Connection ~ 6650 6200
Connection ~ 4600 5450
Wire Wire Line
	2300 5650 2300 7000
Wire Wire Line
	2300 5650 2350 5650
Wire Wire Line
	2800 5650 2750 5650
Wire Wire Line
	2800 5300 2800 5700
Connection ~ 2800 5650
Wire Wire Line
	2800 7000 2800 6950
Wire Wire Line
	3100 6950 3100 7100
Wire Wire Line
	1650 6100 2350 6100
Wire Wire Line
	2150 6350 2350 6350
Wire Wire Line
	1650 6550 2350 6550
Wire Wire Line
	5450 6000 6250 6000
Wire Wire Line
	5450 6000 5450 6200
Connection ~ 5450 6200
Wire Wire Line
	6250 6100 6050 6100
Connection ~ 6050 3100
Wire Wire Line
	5850 3000 5850 2750
Connection ~ 5850 3000
Wire Wire Line
	5800 2000 5850 2000
Wire Wire Line
	5850 1750 5850 2250
Wire Wire Line
	6050 1750 6050 2250
Wire Wire Line
	6050 1800 5800 1800
Wire Wire Line
	5400 2300 5400 2350
Wire Wire Line
	2750 2450 2800 2450
Connection ~ 2800 2450
Wire Wire Line
	2200 1900 2200 3350
Wire Wire Line
	2200 2450 2350 2450
Wire Wire Line
	4800 1900 4750 1900
Wire Wire Line
	4250 1900 2200 1900
Connection ~ 2200 2450
Wire Wire Line
	2750 5450 2800 5450
Connection ~ 2800 5450
Wire Wire Line
	2350 5450 2200 5450
Wire Wire Line
	2200 4900 2200 6350
Wire Wire Line
	5800 5000 5850 5000
Wire Wire Line
	5850 4750 5850 5300
Wire Wire Line
	6050 4750 6050 5300
Wire Wire Line
	6050 4800 5800 4800
Wire Wire Line
	5400 5300 5400 5350
Wire Wire Line
	4800 4900 4750 4900
Wire Wire Line
	4250 4900 2200 4900
Connection ~ 2200 5450
$Comp
L R R210
U 1 1 547B1E15
P 5850 5550
F 0 "R210" V 5930 5550 40  0000 C CNN
F 1 "10k" V 5857 5551 40  0000 C CNN
F 2 "~" V 5780 5550 30  0000 C CNN
F 3 "~" H 5850 5550 30  0000 C CNN
	1    5850 5550
	1    0    0    -1  
$EndComp
$Comp
L R R212
U 1 1 547B1E1B
P 6050 5550
F 0 "R212" V 6130 5550 40  0000 C CNN
F 1 "10k" V 6057 5551 40  0000 C CNN
F 2 "~" V 5980 5550 30  0000 C CNN
F 3 "~" H 6050 5550 30  0000 C CNN
	1    6050 5550
	1    0    0    -1  
$EndComp
Connection ~ 6050 6100
Wire Wire Line
	5850 5800 5850 6000
Connection ~ 5850 6000
Wire Wire Line
	5400 1100 5400 1500
$Comp
L C C213
U 1 1 547B236E
P 5150 1300
F 0 "C213" H 5150 1400 40  0000 L CNN
F 1 "100n" H 5156 1215 40  0000 L CNN
F 2 "~" H 5188 1150 30  0000 C CNN
F 3 "~" H 5150 1300 60  0000 C CNN
	1    5150 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4850 1300 4950 1300
Wire Wire Line
	5350 1300 6050 1300
Connection ~ 5400 1300
$Comp
L DIODE D203
U 1 1 547B2520
P 2550 2450
F 0 "D203" H 2550 2550 40  0000 C CNN
F 1 "DIODE" H 2550 2350 40  0000 C CNN
F 2 "~" H 2550 2450 60  0000 C CNN
F 3 "~" H 2550 2450 60  0000 C CNN
	1    2550 2450
	1    0    0    -1  
$EndComp
$Comp
L DIODE D204
U 1 1 547B2537
P 2550 5450
F 0 "D204" H 2550 5550 40  0000 C CNN
F 1 "DIODE" H 2550 5350 40  0000 C CNN
F 2 "~" H 2550 5450 60  0000 C CNN
F 3 "~" H 2550 5450 60  0000 C CNN
	1    2550 5450
	1    0    0    -1  
$EndComp
$Comp
L DIODE D206
U 1 1 547B2547
P 3350 5650
F 0 "D206" H 3350 5750 40  0000 C CNN
F 1 "DIODE" H 3350 5550 40  0000 C CNN
F 2 "~" H 3350 5650 60  0000 C CNN
F 3 "~" H 3350 5650 60  0000 C CNN
	1    3350 5650
	1    0    0    -1  
$EndComp
$Comp
L DIODE D205
U 1 1 547B254D
P 3350 2650
F 0 "D205" H 3350 2750 40  0000 C CNN
F 1 "DIODE" H 3350 2550 40  0000 C CNN
F 2 "~" H 3350 2650 60  0000 C CNN
F 3 "~" H 3350 2650 60  0000 C CNN
	1    3350 2650
	1    0    0    -1  
$EndComp
$Comp
L DIODE D209
U 1 1 547B2553
P 5850 1550
F 0 "D209" H 5850 1650 40  0000 C CNN
F 1 "DIODE" H 5850 1450 40  0000 C CNN
F 2 "~" H 5850 1550 60  0000 C CNN
F 3 "~" H 5850 1550 60  0000 C CNN
	1    5850 1550
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D211
U 1 1 547B2563
P 6050 1550
F 0 "D211" H 6050 1650 40  0000 C CNN
F 1 "DIODE" H 6050 1450 40  0000 C CNN
F 2 "~" H 6050 1550 60  0000 C CNN
F 3 "~" H 6050 1550 60  0000 C CNN
	1    6050 1550
	0    -1   -1   0   
$EndComp
Connection ~ 5850 2000
Connection ~ 6050 1800
Wire Wire Line
	6050 1300 6050 1350
Wire Wire Line
	5850 1300 5850 1350
Connection ~ 5850 1300
$Comp
L DIODE D210
U 1 1 547B2A5F
P 5850 4550
F 0 "D210" H 5850 4650 40  0000 C CNN
F 1 "DIODE" H 5850 4450 40  0000 C CNN
F 2 "~" H 5850 4550 60  0000 C CNN
F 3 "~" H 5850 4550 60  0000 C CNN
	1    5850 4550
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D212
U 1 1 547B2A65
P 6050 4550
F 0 "D212" H 6050 4650 40  0000 C CNN
F 1 "DIODE" H 6050 4450 40  0000 C CNN
F 2 "~" H 6050 4550 60  0000 C CNN
F 3 "~" H 6050 4550 60  0000 C CNN
	1    6050 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 4300 6050 4300
Wire Wire Line
	5850 4300 5850 4350
Wire Wire Line
	6050 4300 6050 4350
Connection ~ 5850 4300
Connection ~ 5850 5000
Connection ~ 6050 4800
$Comp
L DIODE D201
U 1 1 547B3208
P 1950 3350
F 0 "D201" H 1950 3450 40  0000 C CNN
F 1 "DIODE" H 1950 3250 40  0000 C CNN
F 2 "~" H 1950 3350 60  0000 C CNN
F 3 "~" H 1950 3350 60  0000 C CNN
	1    1950 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 3350 1700 3350
Wire Wire Line
	1700 3350 1700 3550
Connection ~ 1700 3550
Connection ~ 2200 3350
$Comp
L DIODE D202
U 1 1 547B341B
P 1950 6350
F 0 "D202" H 1950 6450 40  0000 C CNN
F 1 "DIODE" H 1950 6250 40  0000 C CNN
F 2 "~" H 1950 6350 60  0000 C CNN
F 3 "~" H 1950 6350 60  0000 C CNN
	1    1950 6350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 6350 1700 6350
Wire Wire Line
	1700 6350 1700 6550
Connection ~ 1700 6550
Connection ~ 2200 6350
Text HLabel 1650 2150 0    60   Input ~ 0
VDD_LOGIC
Wire Wire Line
	2800 2150 1650 2150
Text HLabel 1650 1800 0    60   Input ~ 0
VDD_PWR
Wire Wire Line
	1650 1800 3200 1800
Wire Wire Line
	5400 1100 3100 1100
Connection ~ 3100 1800
Wire Wire Line
	5400 4500 5400 4300
Text Label 3200 1800 0    60   ~ 0
VDD_PWR
Text Label 3100 5050 2    60   ~ 0
VDD_PWR
Wire Wire Line
	3100 5050 3100 5700
Text Label 2800 2050 2    60   ~ 0
VDD_LOGIC
Text Label 2800 5300 2    60   ~ 0
VDD_LOGIC
Connection ~ 2800 2150
Text Label 5300 4300 2    60   ~ 0
VDD_PWR
Connection ~ 5400 4300
Text HLabel 1650 4000 0    60   Input ~ 0
GND_LOGIC
Wire Wire Line
	1650 4000 2800 4000
Text Label 2250 4150 2    60   ~ 0
GND_LOGIC
Text Label 2200 7000 2    60   ~ 0
GND_LOGIC
Wire Wire Line
	2200 7000 2800 7000
Connection ~ 2300 4000
Wire Wire Line
	2300 4150 2250 4150
Connection ~ 2300 7000
Wire Wire Line
	3100 4000 6650 4000
Connection ~ 4600 4000
Wire Wire Line
	6050 4000 6050 3800
Connection ~ 4800 4000
Wire Wire Line
	6250 4000 6250 3800
Connection ~ 6050 4000
Wire Wire Line
	6450 4000 6450 3800
Connection ~ 6250 4000
Wire Wire Line
	6650 4000 6650 3800
Connection ~ 6450 4000
Wire Wire Line
	4800 3800 4800 4000
Wire Wire Line
	3100 7000 6650 7000
Wire Wire Line
	6250 7000 6250 6800
Connection ~ 6050 7000
Wire Wire Line
	6450 7000 6450 6800
Connection ~ 6250 7000
Wire Wire Line
	6650 7000 6650 6800
Connection ~ 6450 7000
Connection ~ 4600 7000
Connection ~ 4800 7000
Text HLabel 1650 4300 0    60   Input ~ 0
GND_PWR
Wire Wire Line
	1650 4300 3200 4300
Connection ~ 3100 4000
Text Label 3200 4300 0    60   ~ 0
GND_PWR
Connection ~ 3100 4300
Text Label 3050 7100 2    60   ~ 0
GND_PWR
Wire Wire Line
	3100 7100 3050 7100
Connection ~ 3100 7000
Text Label 4800 6050 0    60   ~ 0
GND_PWR
Wire Wire Line
	4750 6050 4800 6050
Connection ~ 4750 5950
Text Label 4800 3050 0    60   ~ 0
GND_PWR
Wire Wire Line
	4750 3050 4800 3050
Connection ~ 4750 2950
Text Label 3700 5350 0    60   ~ 0
GND_PWR
Wire Wire Line
	3600 5150 3600 5350
Text Label 5350 5350 2    60   ~ 0
GND_PWR
Wire Wire Line
	5400 5350 5350 5350
Text Label 5350 2350 2    60   ~ 0
GND_PWR
Wire Wire Line
	5400 2350 5350 2350
Text Label 4850 1300 2    60   ~ 0
GND_PWR
Text Label 3700 2350 0    60   ~ 0
GND_PWR
Wire Wire Line
	3600 2150 3600 2350
Text Label 4250 3200 0    60   ~ 0
VSA
Text Label 4250 6200 0    60   ~ 0
VSB
$EndSCHEMATC
