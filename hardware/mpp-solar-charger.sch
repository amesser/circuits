EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:mpp-solar-charger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3900 1650 1000 800 
U 5B7DA76C
F0 "Controller" 60
F1 "controller.sch" 60
F2 "VBAT" I L 3900 1750 60 
F3 "~SWITCH" O L 3900 1900 60 
F4 "USENSE" I L 3900 2350 60 
F5 "UREF" I L 3900 2250 60 
F6 "USET" O L 3900 2150 60 
F7 "+5V" O R 4900 1850 60 
F8 "CHARGED" O R 4900 1750 60 
$EndSheet
$Comp
L GND #PWR01
U 1 1 5B8727CF
P 5550 2900
F 0 "#PWR01" H 5550 2650 50  0001 C CNN
F 1 "GND" H 5550 2750 50  0000 C CNN
F 2 "" H 5550 2900 50  0001 C CNN
F 3 "" H 5550 2900 50  0001 C CNN
	1    5550 2900
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x05 J102
U 1 1 5B872878
P 5850 1950
F 0 "J102" H 5850 2250 50  0000 C CNN
F 1 "Conn_01x05" H 5850 1650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 5850 1950 50  0001 C CNN
F 3 "" H 5850 1950 50  0001 C CNN
	1    5850 1950
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x05 J103
U 1 1 5B8728F8
P 5850 2650
F 0 "J103" H 5850 2950 50  0000 C CNN
F 1 "Conn_01x05" H 5850 2350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 5850 2650 50  0001 C CNN
F 3 "" H 5850 2650 50  0001 C CNN
	1    5850 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1150 5650 1150
Wire Wire Line
	3750 1150 3750 1750
Wire Wire Line
	3750 1750 3900 1750
Wire Wire Line
	3500 1900 3900 1900
Wire Wire Line
	3500 2150 3900 2150
Wire Wire Line
	3500 2250 3900 2250
Wire Wire Line
	3500 2350 3900 2350
Wire Wire Line
	2450 1550 2350 1550
Wire Wire Line
	2350 1550 2350 1350
Wire Wire Line
	4900 1750 5650 1750
Wire Wire Line
	5550 1950 5550 2200
Wire Wire Line
	5550 1950 5650 1950
Wire Wire Line
	4900 1850 5650 1850
Wire Wire Line
	5250 1150 5250 2050
Wire Wire Line
	5250 2050 5650 2050
Connection ~ 5250 1150
Wire Wire Line
	5300 1350 5300 2150
Wire Wire Line
	5300 2150 5650 2150
Connection ~ 5300 1350
Wire Wire Line
	3850 2350 3850 2550
Wire Wire Line
	3850 2550 5650 2550
Connection ~ 3850 2350
Wire Wire Line
	3750 2250 3750 2750
Connection ~ 3750 2250
Wire Wire Line
	3750 2750 5650 2750
Wire Wire Line
	5550 2850 5650 2850
Wire Wire Line
	5550 2450 5550 2900
Wire Wire Line
	5650 2650 5550 2650
Connection ~ 5550 2850
Wire Wire Line
	5650 2450 5550 2450
Connection ~ 5550 2650
$Comp
L GND #PWR02
U 1 1 5B8718B4
P 5550 2200
F 0 "#PWR02" H 5550 1950 50  0001 C CNN
F 1 "GND" H 5550 2050 50  0000 C CNN
F 2 "" H 5550 2200 50  0001 C CNN
F 3 "" H 5550 2200 50  0001 C CNN
	1    5550 2200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J101
U 1 1 5B873137
P 5850 1250
F 0 "J101" H 5850 1450 50  0000 C CNN
F 1 "Conn_01x04" H 5850 950 50  0000 C CNN
F 2 "mpp-solar-charger:2606-1104_020-000" H 5850 1250 50  0001 C CNN
F 3 "" H 5850 1250 50  0001 C CNN
	1    5850 1250
	1    0    0    -1  
$EndComp
$Sheet
S 2450 1450 1050 1000
U 5B78532E
F0 "Power-Stage" 60
F1 "power-stage.sch" 60
F2 "VBAT" B R 3500 1550 60 
F3 "~SWITCH" U R 3500 1900 60 
F4 "VSOLAR" I L 2450 1550 60 
F5 "V_SENSE" O R 3500 2350 60 
F6 "V_REF" O R 3500 2250 60 
F7 "V_SET" O R 3500 2150 60 
$EndSheet
Wire Wire Line
	3500 1550 3750 1550
Connection ~ 3750 1550
$Comp
L GND #PWR03
U 1 1 5B87374C
P 5550 1500
F 0 "#PWR03" H 5550 1250 50  0001 C CNN
F 1 "GND" H 5550 1350 50  0000 C CNN
F 2 "" H 5550 1500 50  0001 C CNN
F 3 "" H 5550 1500 50  0001 C CNN
	1    5550 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1250 5550 1250
Wire Wire Line
	5550 1250 5550 1500
Wire Wire Line
	5650 1450 5550 1450
Connection ~ 5550 1450
Wire Wire Line
	2350 1350 5650 1350
$EndSCHEMATC
