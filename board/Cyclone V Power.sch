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
LIBS:switches
LIBS:amesser_transistors
LIBS:amesser_microcontroller
LIBS:data-logger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 7
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
L 5CEFA2F23 U301
U 2 1 58D588FB
P 2250 1750
F 0 "U301" H 1750 2550 60  0000 C CNN
F 1 "5CEFA2F23" H 2250 850 60  0000 C CNN
F 2 "board_footprints:BGA_484_Pitch1.0" H 2250 1850 60  0001 C CNN
F 3 "" H 2250 1850 60  0001 C CNN
	2    2250 1750
	1    0    0    -1  
$EndComp
Text Label 650  1050 0    60   ~ 0
VCC_3V30_FPGA
$Comp
L GND #PWR079
U 1 1 58D6655D
P 4950 1650
F 0 "#PWR079" H 4950 1400 50  0001 C CNN
F 1 "GND" H 4950 1500 50  0001 C CNN
F 2 "" H 4950 1650 50  0000 C CNN
F 3 "" H 4950 1650 50  0000 C CNN
	1    4950 1650
	1    0    0    -1  
$EndComp
Text Label 5350 1100 2    60   ~ 0
EXT_IO1
Text Label 3950 1200 0    60   ~ 0
EXT_IO2
Text Label 5350 1200 2    60   ~ 0
EXT_IO3
Text Label 3950 1300 0    60   ~ 0
EXT_IO4
Text Label 5350 1300 2    60   ~ 0
EXT_IO5
Text Label 3950 1400 0    60   ~ 0
EXT_IO6
Text Label 5350 1400 2    60   ~ 0
EXT_IO7
Text Label 3950 1100 0    60   ~ 0
EXT_IO0
Text Label 3700 1050 2    60   ~ 0
EXT_IO0
Text Label 3700 1150 2    60   ~ 0
EXT_IO1
Text Label 3700 1450 2    60   ~ 0
EXT_IO2
Text Label 3700 1550 2    60   ~ 0
EXT_IO3
Text Label 3700 1250 2    60   ~ 0
EXT_IO4
Text Label 3700 1350 2    60   ~ 0
EXT_IO5
Text Label 3700 1650 2    60   ~ 0
EXT_IO6
Text Label 3700 1750 2    60   ~ 0
EXT_IO7
Text GLabel 1350 7250 0    60   Input ~ 0
VCC_3V30_FPGA
Text GLabel 1350 6850 0    60   Input ~ 0
VCC_1V10_FPGA
Text Label 2100 7250 2    60   ~ 0
VCC_3V30_FPGA
Text Label 2100 6850 2    60   ~ 0
VCC_1V10_FPGA
Text Label 3700 1850 2    60   ~ 0
EXT_IO8
Text Label 3700 1950 2    60   ~ 0
EXT_IO9
Text Label 3700 2050 2    60   ~ 0
EXT_IO10
Text Label 3700 2150 2    60   ~ 0
EXT_IO11
Text Label 3700 2250 2    60   ~ 0
EXT_IO12
Text Label 3700 2350 2    60   ~ 0
EXT_IO13
Text Label 3700 2450 2    60   ~ 0
EXT_IO14
Text Label 3700 2550 2    60   ~ 0
EXT_IO15
Text Label 3900 2150 0    60   ~ 0
EXT_IO8
Text Label 5400 2150 2    60   ~ 0
EXT_IO9
Text Label 3900 2250 0    60   ~ 0
EXT_IO10
Text Label 5400 2250 2    60   ~ 0
EXT_IO11
Text Label 3900 2350 0    60   ~ 0
EXT_IO12
Text Label 5400 2350 2    60   ~ 0
EXT_IO13
Text Label 3900 2450 0    60   ~ 0
EXT_IO14
Text Label 5400 2450 2    60   ~ 0
EXT_IO15
$Comp
L 5CEFA2F23 U301
U 1 1 58DB14C8
P 8800 3600
F 0 "U301" H 7050 6600 60  0000 C CNN
F 1 "5CEFA2F23" H 8800 1100 60  0000 C CNN
F 2 "board_footprints:BGA_484_Pitch1.0" H 8800 3700 60  0001 C CNN
F 3 "" H 8800 3700 60  0001 C CNN
	1    8800 3600
	1    0    0    -1  
$EndComp
NoConn ~ 9200 700 
NoConn ~ 9200 800 
NoConn ~ 9200 900 
NoConn ~ 9200 1000
$Comp
L GND #PWR080
U 1 1 58D81456
P 8450 5750
F 0 "#PWR080" H 8450 5500 50  0001 C CNN
F 1 "GND" H 8450 5600 50  0001 C CNN
F 2 "" H 8450 5750 50  0000 C CNN
F 3 "" H 8450 5750 50  0000 C CNN
	1    8450 5750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR081
U 1 1 58D815E1
P 10850 5750
F 0 "#PWR081" H 10850 5500 50  0001 C CNN
F 1 "GND" H 10850 5600 50  0001 C CNN
F 2 "" H 10850 5750 50  0000 C CNN
F 3 "" H 10850 5750 50  0000 C CNN
	1    10850 5750
	1    0    0    -1  
$EndComp
Text GLabel 1350 7050 0    60   Input ~ 0
VCC_2V50_FPGA
Text Label 2100 7050 2    60   ~ 0
VCC_2V50_FPGA
Text Label 5650 4200 0    60   ~ 0
VCCPD_B5A
Text Label 5650 4300 0    60   ~ 0
VCCPD_B5B
Text Label 5650 4500 0    60   ~ 0
VCC_2V50_FPGA
$Comp
L GND #PWR082
U 1 1 58DCA82B
P 6450 6050
F 0 "#PWR082" H 6450 5800 50  0001 C CNN
F 1 "GND" H 6450 5900 50  0001 C CNN
F 2 "" H 6450 6050 50  0000 C CNN
F 3 "" H 6450 6050 50  0000 C CNN
	1    6450 6050
	1    0    0    -1  
$EndComp
Text Label 1700 7500 2    60   ~ 0
VCCPD_B5A
Text GLabel 1150 7500 0    60   Input ~ 0
VCCPD_B5A
Text GLabel 1150 7650 0    60   Input ~ 0
VCCPD_B5B
Text Label 1700 7650 2    60   ~ 0
VCCPD_B5B
$Comp
L R R601
U 1 1 58DCF6B0
P 6750 6400
F 0 "R601" V 6830 6400 50  0000 C CNN
F 1 "2.00k / 1%" V 6750 6400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6680 6400 50  0001 C CNN
F 3 "" H 6750 6400 50  0000 C CNN
	1    6750 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR083
U 1 1 58DCFA56
P 6750 6600
F 0 "#PWR083" H 6750 6350 50  0001 C CNN
F 1 "GND" H 6750 6450 50  0001 C CNN
F 2 "" H 6750 6600 50  0000 C CNN
F 3 "" H 6750 6600 50  0000 C CNN
	1    6750 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  1050 1450 1050
Wire Wire Line
	1400 900  1400 1350
Wire Wire Line
	1400 1150 1450 1150
Connection ~ 1400 1050
Wire Wire Line
	1400 1250 1450 1250
Connection ~ 1400 1150
Wire Wire Line
	1400 1350 1450 1350
Connection ~ 1400 1250
Wire Wire Line
	3950 1100 4400 1100
Wire Wire Line
	3950 1200 4400 1200
Wire Wire Line
	3950 1300 4400 1300
Wire Wire Line
	3950 1400 4400 1400
Wire Wire Line
	4900 1100 5350 1100
Wire Wire Line
	4900 1200 5350 1200
Wire Wire Line
	4900 1300 5350 1300
Wire Wire Line
	4900 1400 5350 1400
Wire Wire Line
	3050 1050 3700 1050
Wire Wire Line
	3050 1150 3700 1150
Wire Wire Line
	3050 1250 3700 1250
Wire Wire Line
	3050 1350 3700 1350
Wire Wire Line
	3050 1450 3700 1450
Wire Wire Line
	3050 1550 3700 1550
Wire Wire Line
	3050 1650 3700 1650
Wire Wire Line
	3050 1750 3700 1750
Wire Wire Line
	1350 7250 2100 7250
Wire Wire Line
	3050 1850 3700 1850
Wire Wire Line
	3050 1950 3700 1950
Wire Wire Line
	3050 2050 3700 2050
Wire Wire Line
	3050 2150 3700 2150
Wire Wire Line
	3050 2250 3700 2250
Wire Wire Line
	3050 2350 3700 2350
Wire Wire Line
	3050 2450 3700 2450
Wire Wire Line
	3050 2550 3700 2550
Wire Wire Line
	4900 2150 5400 2150
Wire Wire Line
	4900 2250 5400 2250
Wire Wire Line
	4900 2350 5400 2350
Wire Wire Line
	4900 2450 5400 2450
Wire Wire Line
	4400 2150 3900 2150
Wire Wire Line
	4400 2250 3900 2250
Wire Wire Line
	4400 2350 3900 2350
Wire Wire Line
	4400 2450 3900 2450
Wire Wire Line
	1350 6850 2100 6850
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
	1350 7050 2100 7050
Connection ~ 6750 700 
Wire Wire Line
	6000 3400 6800 3400
Wire Wire Line
	6750 3500 6800 3500
Wire Wire Line
	6750 3600 6800 3600
Connection ~ 6750 3500
Wire Wire Line
	6000 3700 6800 3700
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
	5650 4500 6800 4500
Wire Wire Line
	6750 4900 6800 4900
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
Connection ~ 6750 4600
Wire Wire Line
	5650 4200 6800 4200
Wire Wire Line
	5650 4300 6800 4300
Wire Wire Line
	6750 4300 6750 4400
Wire Wire Line
	6750 4400 6800 4400
Connection ~ 6750 4300
Connection ~ 6750 4900
Connection ~ 6750 5600
Wire Wire Line
	6000 3300 6800 3300
Wire Wire Line
	1150 7500 1700 7500
Wire Wire Line
	1150 7650 1700 7650
Wire Wire Line
	6750 3400 6750 3600
Wire Wire Line
	6750 3700 6750 4100
Wire Wire Line
	6800 6200 6750 6200
Wire Wire Line
	6750 6200 6750 6250
Wire Wire Line
	6750 6550 6750 6600
Connection ~ 6750 5500
Wire Wire Line
	6350 5600 6350 5650
Wire Wire Line
	6550 5600 6550 5650
Wire Wire Line
	6350 5950 6350 6000
Wire Wire Line
	6550 6000 6550 5950
Connection ~ 6550 5600
Wire Wire Line
	6300 5000 6800 5000
$Comp
L GND #PWR084
U 1 1 59499EEF
P 1400 2000
F 0 "#PWR084" H 1400 1750 50  0001 C CNN
F 1 "GND" H 1400 1850 50  0001 C CNN
F 2 "" H 1400 2000 50  0000 C CNN
F 3 "" H 1400 2000 50  0000 C CNN
	1    1400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1950 1400 1950
Wire Wire Line
	1400 1950 1400 2000
Wire Wire Line
	6000 700  6800 700 
$Comp
L C C606
U 1 1 59ACED03
P 6550 5800
F 0 "C606" V 6300 5600 50  0000 L CNN
F 1 "1.0µF/4.0V" V 6400 5600 50  0000 L CNN
F 2 "AMesser_Capacitors:NFM15" H 6588 5650 50  0001 C CNN
F 3 "" H 6550 5800 50  0000 C CNN
F 4 "NFM15PC105R0G3" H 6550 5800 60  0001 C CNN "Part Number"
	1    6550 5800
	1    0    0    -1  
$EndComp
$Comp
L C C604
U 1 1 59ACEDC3
P 6350 5800
F 0 "C604" V 6100 5600 50  0000 L CNN
F 1 "1.0µF/4.0V" V 6200 5600 50  0000 L CNN
F 2 "AMesser_Capacitors:NFM15" H 6388 5650 50  0001 C CNN
F 3 "" H 6350 5800 50  0000 C CNN
F 4 "NFM15PC105R0G3" H 6350 5800 60  0001 C CNN "Part Number"
	1    6350 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 5000 6750 6100
Wire Wire Line
	6350 6000 6550 6000
Wire Wire Line
	6450 6000 6450 6050
Connection ~ 6450 6000
$Comp
L L_Small L602
U 1 1 59AD17B9
P 6200 5000
F 0 "L602" H 6230 5040 50  0000 L CNN
F 1 "BLM18KG" V 6100 4850 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 6200 5000 50  0001 C CNN
F 3 "" H 6200 5000 50  0000 C CNN
	1    6200 5000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR085
U 1 1 59AD17BF
P 6450 5450
F 0 "#PWR085" H 6450 5200 50  0001 C CNN
F 1 "GND" H 6450 5300 50  0001 C CNN
F 2 "" H 6450 5450 50  0000 C CNN
F 3 "" H 6450 5450 50  0000 C CNN
	1    6450 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 5000 6350 5050
Wire Wire Line
	6550 5000 6550 5050
Connection ~ 6350 5000
Wire Wire Line
	6350 5350 6350 5400
Wire Wire Line
	6550 5400 6550 5350
Connection ~ 6550 5000
$Comp
L C C605
U 1 1 59AD17CC
P 6550 5200
F 0 "C605" V 6300 5000 50  0000 L CNN
F 1 "1.0µF/4.0V" V 6400 5000 50  0000 L CNN
F 2 "AMesser_Capacitors:NFM15" H 6588 5050 50  0001 C CNN
F 3 "" H 6550 5200 50  0000 C CNN
F 4 "NFM15PC105R0G3" H 6550 5200 60  0001 C CNN "Part Number"
	1    6550 5200
	1    0    0    -1  
$EndComp
$Comp
L C C603
U 1 1 59AD17D3
P 6350 5200
F 0 "C603" V 6100 5000 50  0000 L CNN
F 1 "1.0µF/4.0V" V 6200 5000 50  0000 L CNN
F 2 "AMesser_Capacitors:NFM15" H 6388 5050 50  0001 C CNN
F 3 "" H 6350 5200 50  0000 C CNN
F 4 "NFM15PC105R0G3" H 6350 5200 60  0001 C CNN "Part Number"
	1    6350 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 5400 6550 5400
Wire Wire Line
	6450 5400 6450 5450
Connection ~ 6450 5400
Connection ~ 6750 5000
Wire Wire Line
	6050 5000 6100 5000
Connection ~ 6750 4500
Connection ~ 6050 4500
Text Label 6000 3400 0    60   ~ 0
VCC_3V30_FPGA
Text Label 6000 3700 0    60   ~ 0
VCC_2V50_FPGA
Text Label 6000 3300 0    60   ~ 0
VCC_2V50_FPGA
Text Label 6000 700  0    60   ~ 0
VCC_1V10_FPGA
$Comp
L CONN_02X06 P601
U 1 1 59B7CEAC
P 4650 1150
F 0 "P601" H 4650 1500 50  0000 C CNN
F 1 "CONN_02X06" H 4650 800 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Angled_2x06_Pitch2.54mm" H 4650 -50 50  0001 C CNN
F 3 "" H 4650 -50 50  0000 C CNN
	1    4650 1150
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X06 P602
U 1 1 59B7D695
P 4650 2200
F 0 "P602" H 4650 2550 50  0000 C CNN
F 1 "CONN_02X06" H 4650 1850 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Angled_2x06_Pitch2.54mm" H 4650 1000 50  0001 C CNN
F 3 "" H 4650 1000 50  0000 C CNN
	1    4650 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR086
U 1 1 59B7EB86
P 4950 2700
F 0 "#PWR086" H 4950 2450 50  0001 C CNN
F 1 "GND" H 4950 2550 50  0001 C CNN
F 2 "" H 4950 2700 50  0000 C CNN
F 3 "" H 4950 2700 50  0000 C CNN
	1    4950 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 900  3850 1950
Wire Wire Line
	1400 900  4400 900 
Connection ~ 3850 900 
Wire Wire Line
	3850 1950 4400 1950
Wire Wire Line
	4350 900  4350 750 
Wire Wire Line
	4350 750  4950 750 
Wire Wire Line
	4950 750  4950 900 
Wire Wire Line
	4950 900  4900 900 
Connection ~ 4350 900 
Wire Wire Line
	4350 1950 4350 1800
Wire Wire Line
	4350 1800 4950 1800
Wire Wire Line
	4950 1800 4950 1950
Wire Wire Line
	4950 1950 4900 1950
Connection ~ 4350 1950
Wire Wire Line
	4400 1000 4350 1000
Wire Wire Line
	4350 1000 4350 1600
Wire Wire Line
	4350 1600 4950 1600
Wire Wire Line
	4950 1000 4950 1650
Wire Wire Line
	4950 1000 4900 1000
Connection ~ 4950 1600
Wire Wire Line
	4900 2050 4950 2050
Wire Wire Line
	4950 2050 4950 2700
Wire Wire Line
	4950 2650 4350 2650
Wire Wire Line
	4350 2650 4350 2050
Wire Wire Line
	4350 2050 4400 2050
Connection ~ 4950 2650
Wire Wire Line
	6050 4500 6050 5000
Wire Wire Line
	6350 5600 6800 5600
$EndSCHEMATC
