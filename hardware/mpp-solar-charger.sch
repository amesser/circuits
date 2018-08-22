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
$Sheet
S 3900 1650 800  750 
U 5B7DA76C
F0 "Controller" 60
F1 "controller.sch" 60
F2 "VBAT" I L 3900 1750 60 
F3 "~SWITCH" O L 3900 1900 60 
F4 "USENSE" I L 3900 2350 60 
F5 "UREF" I L 3900 2250 60 
F6 "USET" O L 3900 2150 60 
$EndSheet
Wire Wire Line
	3500 1550 4900 1550
Wire Wire Line
	3750 1550 3750 1750
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
Connection ~ 3750 1550
Wire Wire Line
	2450 1550 2350 1550
Wire Wire Line
	2350 1550 2350 1350
Wire Wire Line
	2350 1350 4900 1350
$EndSCHEMATC
