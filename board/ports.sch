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
LIBS:data-logger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 8
Title "Altera MAX 10 FPGA Development Board"
Date "2017-01-14"
Rev "1"
Comp "Copyright (c) 2017 Andreas Messer"
Comment1 "Extension Ports"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X10 P?
U 1 1 58C7C285
P 8550 1750
F 0 "P?" H 8550 2300 50  0000 C CNN
F 1 "CONN_02X10" V 8550 1750 50  0000 C CNN
F 2 "" H 8550 550 50  0000 C CNN
F 3 "" H 8550 550 50  0000 C CNN
	1    8550 1750
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X10 P?
U 1 1 58C7C3A0
P 8550 2850
F 0 "P?" H 8550 3400 50  0000 C CNN
F 1 "CONN_02X10" V 8550 2850 50  0000 C CNN
F 2 "" H 8550 1650 50  0000 C CNN
F 3 "" H 8550 1650 50  0000 C CNN
	1    8550 2850
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X10 P?
U 1 1 58C7C4F8
P 8550 3950
F 0 "P?" H 8550 4500 50  0000 C CNN
F 1 "CONN_02X10" V 8550 3950 50  0000 C CNN
F 2 "" H 8550 2750 50  0000 C CNN
F 3 "" H 8550 2750 50  0000 C CNN
	1    8550 3950
	1    0    0    -1  
$EndComp
$Comp
L 5CEFA2F23 U?
U 5 1 58C7CE9E
P 2300 3600
F 0 "U?" H 1800 6000 60  0000 C CNN
F 1 "5CEFA2F23" H 2300 1100 60  0000 C CNN
F 2 "" H 2300 3700 60  0001 C CNN
F 3 "" H 2300 3700 60  0001 C CNN
	5    2300 3600
	1    0    0    -1  
$EndComp
$Comp
L 5CEFA2F23 U?
U 6 1 58C7CF2D
P 5100 2000
F 0 "U?" H 4600 4400 60  0000 C CNN
F 1 "5CEFA2F23" H 5100 -700 60  0000 C CNN
F 2 "" H 5100 2100 60  0001 C CNN
F 3 "" H 5100 2100 60  0001 C CNN
	6    5100 2000
	1    0    0    -1  
$EndComp
$Comp
L 5CEFA2F23 U?
U 7 1 58C7CFB1
P 5100 3600
F 0 "U?" H 4600 4000 60  0000 C CNN
F 1 "5CEFA2F23" H 5100 1100 60  0000 C CNN
F 2 "" H 5100 3700 60  0001 C CNN
F 3 "" H 5100 3700 60  0001 C CNN
	7    5100 3600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7BAD0
P 4050 1500
F 0 "C?" H 4075 1600 50  0000 L CNN
F 1 "4µ7" H 4075 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4088 1350 50  0001 C CNN
F 3 "" H 4050 1500 50  0000 C CNN
	1    4050 1500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7BC00
P 3850 1500
F 0 "C?" H 3875 1600 50  0000 L CNN
F 1 "4µ7" H 3875 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3888 1350 50  0001 C CNN
F 3 "" H 3850 1500 50  0000 C CNN
	1    3850 1500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7BD22
P 4050 3100
F 0 "C?" H 4075 3200 50  0000 L CNN
F 1 "4µ7" H 4075 3000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4088 2950 50  0001 C CNN
F 3 "" H 4050 3100 50  0000 C CNN
	1    4050 3100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7BDE0
P 3850 3100
F 0 "C?" H 3875 3200 50  0000 L CNN
F 1 "4µ7" H 3875 3000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3888 2950 50  0001 C CNN
F 3 "" H 3850 3100 50  0000 C CNN
	1    3850 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58C7BE54
P 3850 3350
F 0 "#PWR?" H 3850 3100 50  0001 C CNN
F 1 "GND" H 3850 3200 50  0000 C CNN
F 2 "" H 3850 3350 50  0000 C CNN
F 3 "" H 3850 3350 50  0000 C CNN
	1    3850 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2900 4300 2900
Wire Wire Line
	4050 2900 4050 2950
Wire Wire Line
	3850 2900 3850 2950
Connection ~ 4050 2900
Wire Wire Line
	4300 3000 4250 3000
Wire Wire Line
	4250 3000 4250 2900
Connection ~ 4250 2900
Wire Wire Line
	3850 3250 3850 3350
Wire Wire Line
	3850 3300 4050 3300
Wire Wire Line
	4050 3300 4050 3250
Connection ~ 3850 3300
Wire Wire Line
	4050 1350 4050 1300
Wire Wire Line
	3750 1300 4300 1300
Wire Wire Line
	3850 1300 3850 1350
Connection ~ 4050 1300
Wire Wire Line
	4300 1400 4250 1400
Wire Wire Line
	4250 1400 4250 1300
Connection ~ 4250 1300
Wire Wire Line
	3850 1650 3850 1750
Wire Wire Line
	3850 1700 4050 1700
Wire Wire Line
	4050 1700 4050 1650
$Comp
L GND #PWR?
U 1 1 58C7BF7A
P 3850 1750
F 0 "#PWR?" H 3850 1500 50  0001 C CNN
F 1 "GND" H 3850 1600 50  0000 C CNN
F 2 "" H 3850 1750 50  0000 C CNN
F 3 "" H 3850 1750 50  0000 C CNN
	1    3850 1750
	1    0    0    -1  
$EndComp
Connection ~ 3850 1700
$Comp
L C C?
U 1 1 58C7CAB6
P 1250 1500
F 0 "C?" H 1275 1600 50  0000 L CNN
F 1 "4µ7" H 1275 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1288 1350 50  0001 C CNN
F 3 "" H 1250 1500 50  0000 C CNN
	1    1250 1500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7CB3F
P 1050 1500
F 0 "C?" H 1075 1600 50  0000 L CNN
F 1 "4µ7" H 1075 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1088 1350 50  0001 C CNN
F 3 "" H 1050 1500 50  0000 C CNN
	1    1050 1500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7CB77
P 850 1500
F 0 "C?" H 875 1600 50  0000 L CNN
F 1 "4µ7" H 875 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 888 1350 50  0001 C CNN
F 3 "" H 850 1500 50  0000 C CNN
	1    850  1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1350 1250 1300
Wire Wire Line
	750  1300 1500 1300
Wire Wire Line
	1050 1300 1050 1350
Connection ~ 1250 1300
Wire Wire Line
	850  1300 850  1350
Connection ~ 1050 1300
Wire Wire Line
	1450 1300 1450 1800
Wire Wire Line
	1450 1400 1500 1400
Connection ~ 1450 1300
Wire Wire Line
	1450 1500 1500 1500
Connection ~ 1450 1400
Wire Wire Line
	1450 1600 1500 1600
Connection ~ 1450 1500
Wire Wire Line
	1450 1700 1500 1700
Connection ~ 1450 1600
Wire Wire Line
	1450 1800 1500 1800
Connection ~ 1450 1700
Wire Wire Line
	1250 1700 1250 1650
Wire Wire Line
	850  1700 1250 1700
Wire Wire Line
	1050 1700 1050 1650
Wire Wire Line
	850  1650 850  1750
Connection ~ 1050 1700
$Comp
L GND #PWR?
U 1 1 58C7D9BA
P 850 1750
F 0 "#PWR?" H 850 1500 50  0001 C CNN
F 1 "GND" H 850 1600 50  0001 C CNN
F 2 "" H 850 1750 50  0000 C CNN
F 3 "" H 850 1750 50  0000 C CNN
	1    850  1750
	1    0    0    -1  
$EndComp
Connection ~ 850  1700
$Comp
L C C?
U 1 1 58C7E86D
P 1250 2200
F 0 "C?" H 1275 2300 50  0000 L CNN
F 1 "4µ7" H 1275 2100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1288 2050 50  0001 C CNN
F 3 "" H 1250 2200 50  0000 C CNN
	1    1250 2200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58C7E9FB
P 1250 2600
F 0 "C?" H 1275 2700 50  0000 L CNN
F 1 "4µ7" H 1275 2500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1288 2450 50  0001 C CNN
F 3 "" H 1250 2600 50  0000 C CNN
	1    1250 2600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58C7F987
P 1050 2200
F 0 "R?" V 1130 2200 50  0000 C CNN
F 1 "R" V 1050 2200 50  0000 C CNN
F 2 "" V 980 2200 50  0000 C CNN
F 3 "" H 1050 2200 50  0000 C CNN
	1    1050 2200
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 58C7FA02
P 1050 2600
F 0 "R?" V 1130 2600 50  0000 C CNN
F 1 "R" V 1050 2600 50  0000 C CNN
F 2 "" V 980 2600 50  0000 C CNN
F 3 "" H 1050 2600 50  0000 C CNN
	1    1050 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 2050 1050 2000
Wire Wire Line
	1050 2000 1250 2000
Wire Wire Line
	1250 2000 1250 2050
Wire Wire Line
	1050 2350 1050 2450
Wire Wire Line
	1050 2400 1250 2400
Wire Wire Line
	1250 2350 1250 2450
Connection ~ 1050 2400
Connection ~ 1250 2400
Wire Wire Line
	1050 2750 1050 2850
Wire Wire Line
	1050 2800 1250 2800
Wire Wire Line
	1250 2800 1250 2750
$Comp
L GND #PWR?
U 1 1 58C7FE84
P 1050 2850
F 0 "#PWR?" H 1050 2600 50  0001 C CNN
F 1 "GND" H 1050 2700 50  0001 C CNN
F 2 "" H 1050 2850 50  0000 C CNN
F 3 "" H 1050 2850 50  0000 C CNN
	1    1050 2850
	1    0    0    -1  
$EndComp
Connection ~ 1050 2800
Text Label 8800 1600 0    60   ~ 0
IO_A1_N
Text Label 8800 1700 0    60   ~ 0
IO_A2_N
Text Label 8800 1800 0    60   ~ 0
IO_A3_N
Text Label 8800 1900 0    60   ~ 0
IO_A4_N
Text Label 8800 2000 0    60   ~ 0
IO_A5_N
Text Label 8800 2100 0    60   ~ 0
IO_A6_N
Text Label 8800 2200 0    60   ~ 0
IO_A7_N
Text Label 8800 1500 0    60   ~ 0
IO_A0_N
Text Label 8800 2700 0    60   ~ 0
IO_B1_N
Text Label 8800 2800 0    60   ~ 0
IO_B2_N
Text Label 8800 2900 0    60   ~ 0
IO_B3_N
Text Label 8800 3000 0    60   ~ 0
IO_B4_N
Text Label 8800 3100 0    60   ~ 0
IO_B5_N
Text Label 8800 3200 0    60   ~ 0
IO_B6_N
Text Label 8800 3300 0    60   ~ 0
IO_B7_N
Text Label 8800 2600 0    60   ~ 0
IO_B0_N
Text Label 8800 3700 0    60   ~ 0
IO_C0_N
Text Label 8800 3800 0    60   ~ 0
IO_C1_N
Text Label 8800 3900 0    60   ~ 0
IO_C2_N
Text Label 8800 4000 0    60   ~ 0
IO_C3_N
Text Label 8800 4100 0    60   ~ 0
IO_C4_N
Text Label 8800 4200 0    60   ~ 0
IO_C5_N
Text Label 8800 4300 0    60   ~ 0
IO_C6_N
Text Label 8800 4400 0    60   ~ 0
IO_C7_N
Text Label 8300 2700 2    60   ~ 0
IO_B1_P
Text Label 8300 2800 2    60   ~ 0
IO_B2_P
Text Label 8300 2900 2    60   ~ 0
IO_B3_P
Text Label 8300 3000 2    60   ~ 0
IO_B4_P
Text Label 8300 3100 2    60   ~ 0
IO_B5_P
Text Label 8300 3200 2    60   ~ 0
IO_B6_P
Text Label 8300 3300 2    60   ~ 0
IO_B7_P
Text Label 8300 2600 2    60   ~ 0
IO_B0_P
Text Label 8300 3700 2    60   ~ 0
IO_C0_P
Text Label 8300 3800 2    60   ~ 0
IO_C1_P
Text Label 8300 3900 2    60   ~ 0
IO_C2_P
Text Label 8300 4000 2    60   ~ 0
IO_C3_P
Text Label 8300 4100 2    60   ~ 0
IO_C4_P
Text Label 8300 4200 2    60   ~ 0
IO_C5_P
Text Label 8300 4300 2    60   ~ 0
IO_C6_P
Text Label 8300 4400 2    60   ~ 0
IO_C7_P
Text Label 8300 1600 2    60   ~ 0
IO_A1_P
Text Label 8300 1700 2    60   ~ 0
IO_A2_P
Text Label 8300 1800 2    60   ~ 0
IO_A3_P
Text Label 8300 1900 2    60   ~ 0
IO_A4_P
Text Label 8300 2000 2    60   ~ 0
IO_A5_P
Text Label 8300 2100 2    60   ~ 0
IO_A6_P
Text Label 8300 2200 2    60   ~ 0
IO_A7_P
Text Label 8300 1500 2    60   ~ 0
IO_A0_P
Text Label 8300 1300 2    60   ~ 0
VCC_3V30
Text Label 8300 2400 2    60   ~ 0
VCC_3V30
Text Label 8300 3500 2    60   ~ 0
VCC_3V30
Text Label 8800 1400 0    60   ~ 0
VCC_2V50
Text Label 8800 2500 0    60   ~ 0
VCC_2V50
Text Label 8800 3600 0    60   ~ 0
VCC_2V50
Text Label 8800 1300 0    60   ~ 0
VCCIO_IO_A
Text Label 8800 2400 0    60   ~ 0
VCC_IO_B
Text Label 8800 3500 0    60   ~ 0
VCC_IO_C
$Comp
L R_ARRAY_4 R?
U 1 1 58C83E99
P 7700 1500
F 0 "R?" V 7750 1800 60  0000 C CNN
F 1 "51R" V 7700 1500 60  0000 C CNN
F 2 "" H 7700 1500 60  0001 C CNN
F 3 "" H 7700 1500 60  0001 C CNN
	1    7700 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7850 1500 8300 1500
Wire Wire Line
	7850 1600 8300 1600
Wire Wire Line
	7850 1700 8300 1700
Wire Wire Line
	7850 1800 8300 1800
Wire Wire Line
	7850 1900 8300 1900
Wire Wire Line
	7850 2000 8300 2000
Wire Wire Line
	7850 2100 8300 2100
Wire Wire Line
	7850 2200 8300 2200
$Comp
L R_ARRAY_4 R?
U 1 1 58C841B1
P 7700 1600
F 0 "R?" V 7750 1900 60  0000 C CNN
F 1 "51R" V 7700 1600 60  0000 C CNN
F 2 "" H 7700 1600 60  0001 C CNN
F 3 "" H 7700 1600 60  0001 C CNN
	1    7700 1600
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C841EB
P 7700 1700
F 0 "R?" V 7750 2000 60  0000 C CNN
F 1 "51R" V 7700 1700 60  0000 C CNN
F 2 "" H 7700 1700 60  0001 C CNN
F 3 "" H 7700 1700 60  0001 C CNN
	1    7700 1700
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8424E
P 7700 1800
F 0 "R?" V 7750 2100 60  0000 C CNN
F 1 "51R" V 7700 1800 60  0000 C CNN
F 2 "" H 7700 1800 60  0001 C CNN
F 3 "" H 7700 1800 60  0001 C CNN
	1    7700 1800
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8431F
P 7700 1900
F 0 "R?" V 7750 2200 60  0000 C CNN
F 1 "51R" V 7700 1900 60  0000 C CNN
F 2 "" H 7700 1900 60  0001 C CNN
F 3 "" H 7700 1900 60  0001 C CNN
	1    7700 1900
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C84325
P 7700 2000
F 0 "R?" V 7750 2300 60  0000 C CNN
F 1 "51R" V 7700 2000 60  0000 C CNN
F 2 "" H 7700 2000 60  0001 C CNN
F 3 "" H 7700 2000 60  0001 C CNN
	1    7700 2000
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8432B
P 7700 2100
F 0 "R?" V 7750 2400 60  0000 C CNN
F 1 "51R" V 7700 2100 60  0000 C CNN
F 2 "" H 7700 2100 60  0001 C CNN
F 3 "" H 7700 2100 60  0001 C CNN
	1    7700 2100
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C84331
P 7700 2200
F 0 "R?" V 7750 2500 60  0000 C CNN
F 1 "51R" V 7700 2200 60  0000 C CNN
F 2 "" H 7700 2200 60  0001 C CNN
F 3 "" H 7700 2200 60  0001 C CNN
	1    7700 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7850 2600 8300 2600
Wire Wire Line
	7850 2700 8300 2700
Wire Wire Line
	7850 2800 8300 2800
Wire Wire Line
	7850 2900 8300 2900
Wire Wire Line
	7850 3000 8300 3000
Wire Wire Line
	7850 3100 8300 3100
Wire Wire Line
	7850 3200 8300 3200
Wire Wire Line
	7850 3300 8300 3300
Wire Wire Line
	7850 3700 8300 3700
Wire Wire Line
	7850 3800 8300 3800
Wire Wire Line
	7850 3900 8300 3900
Wire Wire Line
	7850 4000 8300 4000
Wire Wire Line
	7850 4100 8300 4100
Wire Wire Line
	7850 4200 8300 4200
Wire Wire Line
	7850 4300 8300 4300
Wire Wire Line
	7850 4400 8300 4400
$Comp
L R_ARRAY_4 R?
U 1 1 58C85B51
P 9400 1500
F 0 "R?" V 9350 1700 60  0000 C CNN
F 1 "51R" V 9400 1500 60  0000 C CNN
F 2 "" H 9400 1500 60  0001 C CNN
F 3 "" H 9400 1500 60  0001 C CNN
	1    9400 1500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85C77
P 9400 1600
F 0 "R?" V 9350 1800 60  0000 C CNN
F 1 "51R" V 9400 1600 60  0000 C CNN
F 2 "" H 9400 1600 60  0001 C CNN
F 3 "" H 9400 1600 60  0001 C CNN
	1    9400 1600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85D08
P 9400 1700
F 0 "R?" V 9350 1900 60  0000 C CNN
F 1 "51R" V 9400 1700 60  0000 C CNN
F 2 "" H 9400 1700 60  0001 C CNN
F 3 "" H 9400 1700 60  0001 C CNN
	1    9400 1700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85D9C
P 9400 1800
F 0 "R?" V 9350 2000 60  0000 C CNN
F 1 "51R" V 9400 1800 60  0000 C CNN
F 2 "" H 9400 1800 60  0001 C CNN
F 3 "" H 9400 1800 60  0001 C CNN
	1    9400 1800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85F01
P 9400 1900
F 0 "R?" V 9350 2100 60  0000 C CNN
F 1 "51R" V 9400 1900 60  0000 C CNN
F 2 "" H 9400 1900 60  0001 C CNN
F 3 "" H 9400 1900 60  0001 C CNN
	1    9400 1900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85F07
P 9400 2000
F 0 "R?" V 9350 2200 60  0000 C CNN
F 1 "51R" V 9400 2000 60  0000 C CNN
F 2 "" H 9400 2000 60  0001 C CNN
F 3 "" H 9400 2000 60  0001 C CNN
	1    9400 2000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85F0D
P 9400 2100
F 0 "R?" V 9350 2300 60  0000 C CNN
F 1 "51R" V 9400 2100 60  0000 C CNN
F 2 "" H 9400 2100 60  0001 C CNN
F 3 "" H 9400 2100 60  0001 C CNN
	1    9400 2100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C85F13
P 9400 2200
F 0 "R?" V 9350 2400 60  0000 C CNN
F 1 "51R" V 9400 2200 60  0000 C CNN
F 2 "" H 9400 2200 60  0001 C CNN
F 3 "" H 9400 2200 60  0001 C CNN
	1    9400 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	9250 1500 8800 1500
Wire Wire Line
	9250 1600 8800 1600
Wire Wire Line
	9250 1700 8800 1700
Wire Wire Line
	9250 1800 8800 1800
Wire Wire Line
	9250 1900 8800 1900
Wire Wire Line
	9250 2000 8800 2000
Wire Wire Line
	9250 2100 8800 2100
Wire Wire Line
	9250 2200 8800 2200
Wire Wire Line
	9250 2600 8800 2600
Wire Wire Line
	9250 2700 8800 2700
Wire Wire Line
	9250 2800 8800 2800
Wire Wire Line
	9250 2900 8800 2900
Wire Wire Line
	9250 3000 8800 3000
Wire Wire Line
	9250 3100 8800 3100
Wire Wire Line
	9250 3200 8800 3200
Wire Wire Line
	9250 3300 8800 3300
Wire Wire Line
	8800 3700 9250 3700
Wire Wire Line
	8800 3800 9250 3800
Wire Wire Line
	8800 3900 9250 3900
Wire Wire Line
	8800 4000 9250 4000
Wire Wire Line
	8800 4100 9250 4100
Wire Wire Line
	8800 4200 9250 4200
Wire Wire Line
	8800 4300 9250 4300
Wire Wire Line
	8800 4400 9250 4400
Entry Wire Line
	6800 1400 6900 1500
Entry Wire Line
	6800 1500 6900 1600
Entry Wire Line
	6800 1600 6900 1700
Entry Wire Line
	6800 1700 6900 1800
Entry Wire Line
	6800 1800 6900 1900
Entry Wire Line
	6800 1900 6900 2000
Entry Wire Line
	6800 2000 6900 2100
Entry Wire Line
	6800 2100 6900 2200
Entry Wire Line
	10200 1500 10300 1400
Entry Wire Line
	10200 1600 10300 1500
Entry Wire Line
	10200 1700 10300 1600
Entry Wire Line
	10200 1800 10300 1700
Entry Wire Line
	10200 1900 10300 1800
Entry Wire Line
	10200 2000 10300 1900
Entry Wire Line
	10200 2100 10300 2000
Entry Wire Line
	10200 2200 10300 2100
Wire Wire Line
	9550 1500 10200 1500
Wire Wire Line
	9550 1600 10200 1600
Wire Wire Line
	9550 1700 10200 1700
Wire Wire Line
	9550 1800 10200 1800
Wire Wire Line
	9550 1900 10200 1900
Wire Wire Line
	9550 2000 10200 2000
Wire Wire Line
	9550 2100 10200 2100
Wire Wire Line
	9550 2200 10200 2200
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ECE4
P 9400 2600
F 0 "R?" V 9350 2800 60  0000 C CNN
F 1 "51R" V 9400 2600 60  0000 C CNN
F 2 "" H 9400 2600 60  0001 C CNN
F 3 "" H 9400 2600 60  0001 C CNN
	1    9400 2600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ECEA
P 9400 2700
F 0 "R?" V 9350 2900 60  0000 C CNN
F 1 "51R" V 9400 2700 60  0000 C CNN
F 2 "" H 9400 2700 60  0001 C CNN
F 3 "" H 9400 2700 60  0001 C CNN
	1    9400 2700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ECF0
P 9400 2800
F 0 "R?" V 9350 3000 60  0000 C CNN
F 1 "51R" V 9400 2800 60  0000 C CNN
F 2 "" H 9400 2800 60  0001 C CNN
F 3 "" H 9400 2800 60  0001 C CNN
	1    9400 2800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ECF6
P 9400 2900
F 0 "R?" V 9350 3100 60  0000 C CNN
F 1 "51R" V 9400 2900 60  0000 C CNN
F 2 "" H 9400 2900 60  0001 C CNN
F 3 "" H 9400 2900 60  0001 C CNN
	1    9400 2900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ECFC
P 9400 3000
F 0 "R?" V 9350 3200 60  0000 C CNN
F 1 "51R" V 9400 3000 60  0000 C CNN
F 2 "" H 9400 3000 60  0001 C CNN
F 3 "" H 9400 3000 60  0001 C CNN
	1    9400 3000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ED02
P 9400 3100
F 0 "R?" V 9350 3300 60  0000 C CNN
F 1 "51R" V 9400 3100 60  0000 C CNN
F 2 "" H 9400 3100 60  0001 C CNN
F 3 "" H 9400 3100 60  0001 C CNN
	1    9400 3100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ED08
P 9400 3200
F 0 "R?" V 9350 3400 60  0000 C CNN
F 1 "51R" V 9400 3200 60  0000 C CNN
F 2 "" H 9400 3200 60  0001 C CNN
F 3 "" H 9400 3200 60  0001 C CNN
	1    9400 3200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7ED0E
P 9400 3300
F 0 "R?" V 9350 3500 60  0000 C CNN
F 1 "51R" V 9400 3300 60  0000 C CNN
F 2 "" H 9400 3300 60  0001 C CNN
F 3 "" H 9400 3300 60  0001 C CNN
	1    9400 3300
	0    1    1    0   
$EndComp
Entry Wire Line
	10200 2600 10300 2500
Entry Wire Line
	10200 2700 10300 2600
Entry Wire Line
	10200 2800 10300 2700
Entry Wire Line
	10200 2900 10300 2800
Entry Wire Line
	10200 3000 10300 2900
Entry Wire Line
	10200 3100 10300 3000
Entry Wire Line
	10200 3200 10300 3100
Entry Wire Line
	10200 3300 10300 3200
Wire Wire Line
	9550 2600 10200 2600
Wire Wire Line
	9550 2700 10200 2700
Wire Wire Line
	9550 2800 10200 2800
Wire Wire Line
	9550 2900 10200 2900
Wire Wire Line
	9550 3000 10200 3000
Wire Wire Line
	9550 3100 10200 3100
Wire Wire Line
	9550 3200 10200 3200
Wire Wire Line
	9550 3300 10200 3300
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF68
P 9400 3700
F 0 "R?" V 9350 3900 60  0000 C CNN
F 1 "51R" V 9400 3700 60  0000 C CNN
F 2 "" H 9400 3700 60  0001 C CNN
F 3 "" H 9400 3700 60  0001 C CNN
	1    9400 3700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF6E
P 9400 3800
F 0 "R?" V 9350 4000 60  0000 C CNN
F 1 "51R" V 9400 3800 60  0000 C CNN
F 2 "" H 9400 3800 60  0001 C CNN
F 3 "" H 9400 3800 60  0001 C CNN
	1    9400 3800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF74
P 9400 3900
F 0 "R?" V 9350 4100 60  0000 C CNN
F 1 "51R" V 9400 3900 60  0000 C CNN
F 2 "" H 9400 3900 60  0001 C CNN
F 3 "" H 9400 3900 60  0001 C CNN
	1    9400 3900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF7A
P 9400 4000
F 0 "R?" V 9350 4200 60  0000 C CNN
F 1 "51R" V 9400 4000 60  0000 C CNN
F 2 "" H 9400 4000 60  0001 C CNN
F 3 "" H 9400 4000 60  0001 C CNN
	1    9400 4000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF80
P 9400 4100
F 0 "R?" V 9350 4300 60  0000 C CNN
F 1 "51R" V 9400 4100 60  0000 C CNN
F 2 "" H 9400 4100 60  0001 C CNN
F 3 "" H 9400 4100 60  0001 C CNN
	1    9400 4100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF86
P 9400 4200
F 0 "R?" V 9350 4400 60  0000 C CNN
F 1 "51R" V 9400 4200 60  0000 C CNN
F 2 "" H 9400 4200 60  0001 C CNN
F 3 "" H 9400 4200 60  0001 C CNN
	1    9400 4200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF8C
P 9400 4300
F 0 "R?" V 9350 4500 60  0000 C CNN
F 1 "51R" V 9400 4300 60  0000 C CNN
F 2 "" H 9400 4300 60  0001 C CNN
F 3 "" H 9400 4300 60  0001 C CNN
	1    9400 4300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C7EF92
P 9400 4400
F 0 "R?" V 9350 4600 60  0000 C CNN
F 1 "51R" V 9400 4400 60  0000 C CNN
F 2 "" H 9400 4400 60  0001 C CNN
F 3 "" H 9400 4400 60  0001 C CNN
	1    9400 4400
	0    1    1    0   
$EndComp
Entry Wire Line
	10200 3700 10300 3600
Entry Wire Line
	10200 3800 10300 3700
Entry Wire Line
	10200 3900 10300 3800
Entry Wire Line
	10200 4000 10300 3900
Entry Wire Line
	10200 4100 10300 4000
Entry Wire Line
	10200 4200 10300 4100
Entry Wire Line
	10200 4300 10300 4200
Entry Wire Line
	10200 4400 10300 4300
Wire Wire Line
	9550 3700 10200 3700
Wire Wire Line
	9550 3800 10200 3800
Wire Wire Line
	9550 3900 10200 3900
Wire Wire Line
	9550 4000 10200 4000
Wire Wire Line
	9550 4100 10200 4100
Wire Wire Line
	9550 4200 10200 4200
Wire Wire Line
	9550 4300 10200 4300
Wire Wire Line
	9550 4400 10200 4400
Wire Wire Line
	7550 1500 6900 1500
Wire Wire Line
	7550 1600 6900 1600
Wire Wire Line
	7550 1700 6900 1700
Wire Wire Line
	7550 1800 6900 1800
Wire Wire Line
	7550 1900 6900 1900
Wire Wire Line
	7550 2000 6900 2000
Wire Wire Line
	7550 2100 6900 2100
Wire Wire Line
	7550 2200 6900 2200
$Comp
L R_ARRAY_4 R?
U 1 1 58C82304
P 7700 2600
F 0 "R?" V 7750 2900 60  0000 C CNN
F 1 "51R" V 7700 2600 60  0000 C CNN
F 2 "" H 7700 2600 60  0001 C CNN
F 3 "" H 7700 2600 60  0001 C CNN
	1    7700 2600
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8230A
P 7700 2700
F 0 "R?" V 7750 3000 60  0000 C CNN
F 1 "51R" V 7700 2700 60  0000 C CNN
F 2 "" H 7700 2700 60  0001 C CNN
F 3 "" H 7700 2700 60  0001 C CNN
	1    7700 2700
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C82310
P 7700 2800
F 0 "R?" V 7750 3100 60  0000 C CNN
F 1 "51R" V 7700 2800 60  0000 C CNN
F 2 "" H 7700 2800 60  0001 C CNN
F 3 "" H 7700 2800 60  0001 C CNN
	1    7700 2800
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C82316
P 7700 2900
F 0 "R?" V 7750 3200 60  0000 C CNN
F 1 "51R" V 7700 2900 60  0000 C CNN
F 2 "" H 7700 2900 60  0001 C CNN
F 3 "" H 7700 2900 60  0001 C CNN
	1    7700 2900
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8231C
P 7700 3000
F 0 "R?" V 7750 3300 60  0000 C CNN
F 1 "51R" V 7700 3000 60  0000 C CNN
F 2 "" H 7700 3000 60  0001 C CNN
F 3 "" H 7700 3000 60  0001 C CNN
	1    7700 3000
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C82322
P 7700 3100
F 0 "R?" V 7750 3400 60  0000 C CNN
F 1 "51R" V 7700 3100 60  0000 C CNN
F 2 "" H 7700 3100 60  0001 C CNN
F 3 "" H 7700 3100 60  0001 C CNN
	1    7700 3100
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C82328
P 7700 3200
F 0 "R?" V 7750 3500 60  0000 C CNN
F 1 "51R" V 7700 3200 60  0000 C CNN
F 2 "" H 7700 3200 60  0001 C CNN
F 3 "" H 7700 3200 60  0001 C CNN
	1    7700 3200
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8232E
P 7700 3300
F 0 "R?" V 7750 3600 60  0000 C CNN
F 1 "51R" V 7700 3300 60  0000 C CNN
F 2 "" H 7700 3300 60  0001 C CNN
F 3 "" H 7700 3300 60  0001 C CNN
	1    7700 3300
	0    -1   -1   0   
$EndComp
Entry Wire Line
	6800 2500 6900 2600
Entry Wire Line
	6800 2600 6900 2700
Entry Wire Line
	6800 2700 6900 2800
Entry Wire Line
	6800 2800 6900 2900
Entry Wire Line
	6800 2900 6900 3000
Entry Wire Line
	6800 3000 6900 3100
Entry Wire Line
	6800 3100 6900 3200
Entry Wire Line
	6800 3200 6900 3300
Wire Wire Line
	7550 2600 6900 2600
Wire Wire Line
	7550 2700 6900 2700
Wire Wire Line
	7550 2800 6900 2800
Wire Wire Line
	7550 2900 6900 2900
Wire Wire Line
	7550 3000 6900 3000
Wire Wire Line
	7550 3100 6900 3100
Wire Wire Line
	7550 3200 6900 3200
Wire Wire Line
	7550 3300 6900 3300
$Comp
L R_ARRAY_4 R?
U 1 1 58C82698
P 7700 3700
F 0 "R?" V 7750 4000 60  0000 C CNN
F 1 "51R" V 7700 3700 60  0000 C CNN
F 2 "" H 7700 3700 60  0001 C CNN
F 3 "" H 7700 3700 60  0001 C CNN
	1    7700 3700
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C8269E
P 7700 3800
F 0 "R?" V 7750 4100 60  0000 C CNN
F 1 "51R" V 7700 3800 60  0000 C CNN
F 2 "" H 7700 3800 60  0001 C CNN
F 3 "" H 7700 3800 60  0001 C CNN
	1    7700 3800
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826A4
P 7700 3900
F 0 "R?" V 7750 4200 60  0000 C CNN
F 1 "51R" V 7700 3900 60  0000 C CNN
F 2 "" H 7700 3900 60  0001 C CNN
F 3 "" H 7700 3900 60  0001 C CNN
	1    7700 3900
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826AA
P 7700 4000
F 0 "R?" V 7750 4300 60  0000 C CNN
F 1 "51R" V 7700 4000 60  0000 C CNN
F 2 "" H 7700 4000 60  0001 C CNN
F 3 "" H 7700 4000 60  0001 C CNN
	1    7700 4000
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826B0
P 7700 4100
F 0 "R?" V 7750 4400 60  0000 C CNN
F 1 "51R" V 7700 4100 60  0000 C CNN
F 2 "" H 7700 4100 60  0001 C CNN
F 3 "" H 7700 4100 60  0001 C CNN
	1    7700 4100
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826B6
P 7700 4200
F 0 "R?" V 7750 4500 60  0000 C CNN
F 1 "51R" V 7700 4200 60  0000 C CNN
F 2 "" H 7700 4200 60  0001 C CNN
F 3 "" H 7700 4200 60  0001 C CNN
	1    7700 4200
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826BC
P 7700 4300
F 0 "R?" V 7750 4600 60  0000 C CNN
F 1 "51R" V 7700 4300 60  0000 C CNN
F 2 "" H 7700 4300 60  0001 C CNN
F 3 "" H 7700 4300 60  0001 C CNN
	1    7700 4300
	0    -1   -1   0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58C826C2
P 7700 4400
F 0 "R?" V 7750 4700 60  0000 C CNN
F 1 "51R" V 7700 4400 60  0000 C CNN
F 2 "" H 7700 4400 60  0001 C CNN
F 3 "" H 7700 4400 60  0001 C CNN
	1    7700 4400
	0    -1   -1   0   
$EndComp
Entry Wire Line
	6800 3600 6900 3700
Entry Wire Line
	6800 3700 6900 3800
Entry Wire Line
	6800 3800 6900 3900
Entry Wire Line
	6800 3900 6900 4000
Entry Wire Line
	6800 4000 6900 4100
Entry Wire Line
	6800 4100 6900 4200
Entry Wire Line
	6800 4200 6900 4300
Entry Wire Line
	6800 4300 6900 4400
Wire Wire Line
	7550 3700 6900 3700
Wire Wire Line
	7550 3800 6900 3800
Wire Wire Line
	7550 3900 6900 3900
Wire Wire Line
	7550 4000 6900 4000
Wire Wire Line
	7550 4100 6900 4100
Wire Wire Line
	7550 4200 6900 4200
Wire Wire Line
	7550 4300 6900 4300
Wire Wire Line
	7550 4400 6900 4400
Text Label 5900 2900 0    60   ~ 0
FPGA_R9_N
Text Label 5900 3000 0    60   ~ 0
FPGA_R9_P
Text Label 5900 3100 0    60   ~ 0
FPGA_R10_N
Text Label 5900 3200 0    60   ~ 0
FPGA_R10_P
Text Label 5900 3300 0    60   ~ 0
FPGA_R11_N
Text Label 5900 3400 0    60   ~ 0
FPGA_R11_P
Text Label 5900 3500 0    60   ~ 0
FPGA_R12_N
Text Label 5900 3600 0    60   ~ 0
FPGA_R12_P
Text Label 5900 3700 0    60   ~ 0
FPGA_R13_N
Text Label 5900 3800 0    60   ~ 0
FPAG_R13_P
Text Label 5900 3900 0    60   ~ 0
FPGA_R14_N
Text Label 5900 4100 0    60   ~ 0
FPGA_R15_N
Text Label 5900 4000 0    60   ~ 0
FPGA_R14_P
Text Label 5900 4200 0    60   ~ 0
FPGA_R15_P
Text Label 5900 4300 0    60   ~ 0
FPGA_R16_N
Text Label 5900 4400 0    60   ~ 0
FPGA_R16_P
Text Label 5900 1300 0    60   ~ 0
FPGA_R1_N
Text Label 5900 1400 0    60   ~ 0
FPGA_R1_P
Text Label 5900 1500 0    60   ~ 0
FPGA_R2_N
Text Label 5900 1600 0    60   ~ 0
FPGA_R2_P
Text Label 5900 1700 0    60   ~ 0
FPGA_R3_N
Text Label 5900 1800 0    60   ~ 0
FPGA_R3_P
Text Label 5900 1900 0    60   ~ 0
FPGA_R4_N
Text Label 5900 2000 0    60   ~ 0
FPGA_R4_P
Text Label 5900 2100 0    60   ~ 0
FPGA_R5_N
Text Label 5900 2200 0    60   ~ 0
FPGA_R5_P
Text Label 5900 2300 0    60   ~ 0
FPGA_R6_N
Text Label 5900 2400 0    60   ~ 0
FPGA_R6_P
Text Label 5900 2500 0    60   ~ 0
FPGA_R7_N
Text Label 5900 2700 0    60   ~ 0
FPGA_R8_N
Text Label 5900 2600 0    60   ~ 0
FPGA_R7_P
Text Label 5900 2800 0    60   ~ 0
FPGA_R8_P
Text Label 750  1300 0    60   ~ 0
VCCIO_IO_A
Connection ~ 3850 2900
Text Label 3750 1300 0    60   ~ 0
VCC_IO_B
Text Label 3700 2900 0    60   ~ 0
VCC_IO_C
Text Label 10200 2600 2    60   ~ 0
FPGA_R1_N
Text Label 10200 2700 2    60   ~ 0
FPGA_R2_N
Text Label 10200 2800 2    60   ~ 0
FPGA_R3_N
Text Label 10200 2900 2    60   ~ 0
FPGA_R4_N
Text Label 10200 3000 2    60   ~ 0
FPGA_R5_N
Text Label 10200 3100 2    60   ~ 0
FPGA_R6_N
Text Label 10200 3200 2    60   ~ 0
FPGA_R7_N
Text Label 10200 3300 2    60   ~ 0
FPGA_R8_N
Text Label 6900 2600 0    60   ~ 0
FPGA_R1_P
Text Label 6900 2700 0    60   ~ 0
FPGA_R2_P
Text Label 6900 2800 0    60   ~ 0
FPGA_R3_P
Text Label 6900 2900 0    60   ~ 0
FPGA_R4_P
Text Label 6900 3000 0    60   ~ 0
FPGA_R5_P
Text Label 6900 3100 0    60   ~ 0
FPGA_R6_P
Text Label 6900 3200 0    60   ~ 0
FPGA_R7_P
Text Label 6900 3300 0    60   ~ 0
FPGA_R8_P
Text Label 10200 3700 2    60   ~ 0
FPGA_R9_N
Text Label 10200 3800 2    60   ~ 0
FPGA_R10_N
Text Label 10200 3900 2    60   ~ 0
FPGA_R11_N
Text Label 10200 4000 2    60   ~ 0
FPGA_R12_N
Text Label 10200 4100 2    60   ~ 0
FPGA_R13_N
Text Label 10200 4200 2    60   ~ 0
FPGA_R14_N
Text Label 10200 4300 2    60   ~ 0
FPGA_R15_N
Text Label 10200 4400 2    60   ~ 0
FPGA_R16_N
Text Label 6900 3700 0    60   ~ 0
FPGA_R9_P
Text Label 6900 3800 0    60   ~ 0
FPGA_R10_P
Text Label 6900 3900 0    60   ~ 0
FPGA_R11_P
Text Label 6900 4000 0    60   ~ 0
FPGA_R12_P
Text Label 6900 4100 0    60   ~ 0
FPAG_R13_P
Text Label 6900 4200 0    60   ~ 0
FPGA_R14_P
Text Label 6900 4300 0    60   ~ 0
FPGA_R15_P
Text Label 6900 4400 0    60   ~ 0
FPGA_R16_P
Text Label 3100 1300 0    60   ~ 0
FPGA_B25_N
Connection ~ 850  1300
Connection ~ 3850 1300
Text Label 3100 1400 0    60   ~ 0
FPGA_B25_P
Text Label 3100 2500 0    60   ~ 0
FPGA_B31_N
Text Label 3100 2600 0    60   ~ 0
FPGA_B31_P
Text Label 3100 4100 0    60   ~ 0
FPGA_B39_N
Text Label 3100 4200 0    60   ~ 0
FPGA_B39_P
Text Label 3100 1500 0    60   ~ 0
FPGA_B26_N
Text Label 3100 1600 0    60   ~ 0
FPGA_B26_P
Text Label 3100 1700 0    60   ~ 0
FPGA_B27_N
Text Label 3100 1800 0    60   ~ 0
FPGA_B27_P
Text Label 3100 1900 0    60   ~ 0
FPGA_B28_N
Text Label 3100 2000 0    60   ~ 0
FPGA_B28_P
Text Label 3100 2100 0    60   ~ 0
FPGA_B29_N
Text Label 3100 2200 0    60   ~ 0
FPGA_B29_P
Text Label 3100 2300 0    60   ~ 0
FPGA_B30_N
Text Label 3100 2400 0    60   ~ 0
FPGA_B30_P
Text Label 10200 1500 2    60   ~ 0
FPGA_B25_N
Text Label 10200 1600 2    60   ~ 0
FPGA_B26_N
Text Label 10200 1700 2    60   ~ 0
FPGA_B27_N
Text Label 10200 1800 2    60   ~ 0
FPGA_B28_N
Text Label 10200 1900 2    60   ~ 0
FPGA_B29_N
Text Label 10200 2000 2    60   ~ 0
FPGA_B30_N
Text Label 10200 2100 2    60   ~ 0
FPGA_B31_N
Text Label 10200 2200 2    60   ~ 0
FPGA_B39_N
Text Label 6900 1500 0    60   ~ 0
FPGA_B25_P
Text Label 6900 1600 0    60   ~ 0
FPGA_B26_N
Text Label 6900 1700 0    60   ~ 0
FPGA_B27_P
Text Label 6900 1800 0    60   ~ 0
FPGA_B28_P
Text Label 6900 1900 0    60   ~ 0
FPGA_B29_P
Text Label 6900 2000 0    60   ~ 0
FPGA_B30_P
Text Label 6900 2100 0    60   ~ 0
FPGA_B31_P
Text Label 6900 2200 0    60   ~ 0
FPGA_B39_P
$EndSCHEMATC
