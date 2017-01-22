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
Comment1 "Extension Ports"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 10M08DCF256-RESCUE-data-logger U?
U 4 1 586885C6
P 2050 3400
AR Path="/586885C6" Ref="U?"  Part="4" 
AR Path="/586885BD/586885C6" Ref="U201"  Part="4" 
F 0 "U201" H 2050 3400 60  0000 C CNN
F 1 "10M08DCF256" H 2050 1600 60  0000 C CNN
F 2 "board_footprints:FBGA_256" H 2050 2200 60  0001 C CNN
F 3 "" H 2050 2200 60  0001 C CNN
	4    2050 3400
	1    0    0    -1  
$EndComp
$Comp
L 10M08DCF256-RESCUE-data-logger U?
U 5 1 5868865D
P 5950 2400
AR Path="/5868865D" Ref="U?"  Part="5" 
AR Path="/586885BD/5868865D" Ref="U201"  Part="5" 
F 0 "U201" H 5950 2400 60  0000 C CNN
F 1 "10M08DCF256" H 5950 1600 60  0000 C CNN
F 2 "board_footprints:FBGA_256" H 5950 1200 60  0001 C CNN
F 3 "" H 5950 1200 60  0001 C CNN
	5    5950 2400
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P604
U 1 1 5868FC11
P 7550 5650
F 0 "P604" H 7550 6100 50  0000 C CNN
F 1 "CONN_02X08" V 7550 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x08_Pitch2.54mm" H 7550 4450 50  0001 C CNN
F 3 "" H 7550 4450 50  0000 C CNN
	1    7550 5650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR080
U 1 1 5868FF32
P 7400 5300
F 0 "#PWR080" H 7400 5150 50  0001 C CNN
F 1 "+3V3" H 7400 5440 50  0000 C CNN
F 2 "" H 7400 5300 50  0000 C CNN
F 3 "" H 7400 5300 50  0000 C CNN
	1    7400 5300
	1    0    0    -1  
$EndComp
$Comp
L C C602
U 1 1 58691008
P 1150 2100
F 0 "C602" H 1175 2200 50  0000 L CNN
F 1 "4.7µF" H 1175 2000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1188 1950 50  0001 C CNN
F 3 "" H 1150 2100 50  0000 C CNN
	1    1150 2100
	1    0    0    -1  
$EndComp
$Comp
L C C601
U 1 1 586910EA
P 850 2100
F 0 "C601" H 875 2200 50  0000 L CNN
F 1 "4.7µF" H 875 2000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 888 1950 50  0001 C CNN
F 3 "" H 850 2100 50  0000 C CNN
	1    850  2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR081
U 1 1 586911CB
P 1000 2350
F 0 "#PWR081" H 1000 2100 50  0001 C CNN
F 1 "GND" H 1000 2200 50  0001 C CNN
F 2 "" H 1000 2350 50  0000 C CNN
F 3 "" H 1000 2350 50  0000 C CNN
	1    1000 2350
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U601
U 2 1 586958B5
P 6200 1150
F 0 "U601" H 6200 1300 50  0000 L CNN
F 1 "MCP6002" H 6200 1000 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6100 1200 50  0001 C CNN
F 3 "" H 6200 1300 50  0000 C CNN
	2    6200 1150
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR082
U 1 1 58695B52
P 6100 750
F 0 "#PWR082" H 6100 600 50  0001 C CNN
F 1 "+3V3" H 6100 890 50  0000 C CNN
F 2 "" H 6100 750 50  0000 C CNN
F 3 "" H 6100 750 50  0000 C CNN
	1    6100 750 
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U601
U 1 1 58697D0E
P 2300 1150
F 0 "U601" H 2300 1300 50  0000 L CNN
F 1 "MCP6002" H 2300 1000 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2200 1200 50  0001 C CNN
F 3 "" H 2300 1300 50  0000 C CNN
	1    2300 1150
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P601
U 1 1 58691A75
P 3350 5650
F 0 "P601" H 3350 6100 50  0000 C CNN
F 1 "CONN_02X08" V 3350 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x08_Pitch2.54mm" H 3350 4450 50  0001 C CNN
F 3 "" H 3350 4450 50  0000 C CNN
	1    3350 5650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR083
U 1 1 58691CC3
P 3200 5300
F 0 "#PWR083" H 3200 5150 50  0001 C CNN
F 1 "+3V3" H 3200 5440 50  0000 C CNN
F 2 "" H 3200 5300 50  0000 C CNN
F 3 "" H 3200 5300 50  0000 C CNN
	1    3200 5300
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P602
U 1 1 5869218E
P 4650 5650
F 0 "P602" H 4650 6100 50  0000 C CNN
F 1 "CONN_02X08" V 4650 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x08_Pitch2.54mm" H 4650 4450 50  0001 C CNN
F 3 "" H 4650 4450 50  0000 C CNN
	1    4650 5650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR084
U 1 1 58692340
P 4500 5300
F 0 "#PWR084" H 4500 5150 50  0001 C CNN
F 1 "+3V3" H 4500 5440 50  0000 C CNN
F 2 "" H 4500 5300 50  0000 C CNN
F 3 "" H 4500 5300 50  0000 C CNN
	1    4500 5300
	1    0    0    -1  
$EndComp
NoConn ~ 3100 5400
$Comp
L CONN_02X08 P603
U 1 1 5869259B
P 5950 5650
F 0 "P603" H 5950 6100 50  0000 C CNN
F 1 "CONN_02X08" V 5950 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x08_Pitch2.54mm" H 5950 4450 50  0001 C CNN
F 3 "" H 5950 4450 50  0000 C CNN
	1    5950 5650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR085
U 1 1 58692613
P 5800 5300
F 0 "#PWR085" H 5800 5150 50  0001 C CNN
F 1 "+3V3" H 5800 5440 50  0000 C CNN
F 2 "" H 5800 5300 50  0000 C CNN
F 3 "" H 5800 5300 50  0000 C CNN
	1    5800 5300
	1    0    0    -1  
$EndComp
$Comp
L +1V8 #PWR086
U 1 1 586953E8
P 2800 6050
F 0 "#PWR086" H 2800 5900 50  0001 C CNN
F 1 "+1V8" H 2800 6190 50  0000 C CNN
F 2 "" H 2800 6050 50  0000 C CNN
F 3 "" H 2800 6050 50  0000 C CNN
	1    2800 6050
	1    0    0    -1  
$EndComp
$Comp
L +1V2 #PWR087
U 1 1 5869576F
P 2600 6050
F 0 "#PWR087" H 2600 5900 50  0001 C CNN
F 1 "+1V2" H 2600 6190 50  0000 C CNN
F 2 "" H 2600 6050 50  0000 C CNN
F 3 "" H 2600 6050 50  0000 C CNN
	1    2600 6050
	1    0    0    -1  
$EndComp
NoConn ~ 5700 5400
$Comp
L R R601
U 1 1 58697DCC
P 2800 5350
F 0 "R601" V 2700 5350 50  0000 C CNN
F 1 "6.8k" V 2800 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2730 5350 50  0001 C CNN
F 3 "" H 2800 5350 50  0000 C CNN
	1    2800 5350
	0    1    1    0   
$EndComp
$Comp
L C C603
U 1 1 5869970B
P 1700 1300
F 0 "C603" H 1725 1400 50  0000 L CNN
F 1 "100nF" H 1725 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1738 1150 50  0001 C CNN
F 3 "" H 1700 1300 50  0000 C CNN
	1    1700 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 5900 7400 5950
Wire Wire Line
	5300 2000 5350 2000
Wire Wire Line
	4750 1900 5350 1900
Connection ~ 5300 2000
Wire Wire Line
	5050 1450 5050 1950
Connection ~ 5300 1900
Wire Wire Line
	4750 1900 4750 1950
Connection ~ 5050 1900
Wire Wire Line
	7900 5900 8000 5900
Wire Wire Line
	8000 5900 8000 3000
Wire Wire Line
	8000 3000 6950 3000
Wire Wire Line
	6950 2800 8050 2800
Wire Wire Line
	8050 2800 8050 6000
Wire Wire Line
	8050 6000 7800 6000
Wire Wire Line
	7800 6000 7800 5900
Wire Wire Line
	6950 2600 8100 2600
Wire Wire Line
	8100 2600 8100 6050
Wire Wire Line
	8100 6050 7700 6050
Wire Wire Line
	7700 6050 7700 5900
Wire Wire Line
	6950 2400 8150 2400
Wire Wire Line
	8150 2400 8150 6100
Wire Wire Line
	8150 6100 7600 6100
Wire Wire Line
	7600 6100 7600 5900
Wire Wire Line
	6950 2200 8200 2200
Wire Wire Line
	8200 2200 8200 6150
Wire Wire Line
	8200 6150 7500 6150
Wire Wire Line
	7500 6150 7500 5900
Wire Wire Line
	7500 2100 7500 5400
Wire Wire Line
	6950 2100 7500 2100
Wire Wire Line
	7600 2300 7600 5400
Wire Wire Line
	6950 2300 7600 2300
Wire Wire Line
	7700 2500 7700 5400
Wire Wire Line
	6950 2500 7700 2500
Wire Wire Line
	7800 2700 7800 5400
Wire Wire Line
	6950 2700 7800 2700
Wire Wire Line
	7900 2900 7900 5400
Wire Wire Line
	6950 2900 7900 2900
Wire Wire Line
	850  1900 1450 1900
Wire Wire Line
	1400 2100 1450 2100
Wire Wire Line
	1400 2000 1450 2000
Connection ~ 1400 2000
Wire Wire Line
	850  2300 1150 2300
Connection ~ 1400 1900
Wire Wire Line
	1400 5100 4400 5100
Connection ~ 1400 2100
Wire Wire Line
	3300 2100 3300 5400
Wire Wire Line
	3050 2100 3300 2100
Wire Wire Line
	3400 2300 3400 5400
Wire Wire Line
	3050 2300 3400 2300
Wire Wire Line
	3500 2500 3500 5400
Wire Wire Line
	3050 2500 3500 2500
Wire Wire Line
	3600 2700 3600 5400
Wire Wire Line
	3050 2700 3600 2700
Wire Wire Line
	3700 2900 3700 5400
Wire Wire Line
	3050 2900 3700 2900
Wire Wire Line
	3050 3000 3800 3000
Wire Wire Line
	3800 3000 3800 5950
Wire Wire Line
	3800 5950 3700 5950
Wire Wire Line
	3700 5950 3700 5900
Wire Wire Line
	3600 5900 3600 6000
Wire Wire Line
	3600 6000 3850 6000
Wire Wire Line
	3850 6000 3850 2800
Wire Wire Line
	3850 2800 3050 2800
Wire Wire Line
	3050 2600 3900 2600
Wire Wire Line
	3900 2600 3900 6050
Wire Wire Line
	3900 6050 3500 6050
Wire Wire Line
	3500 6050 3500 5900
Wire Wire Line
	3400 5900 3400 6100
Wire Wire Line
	3400 6100 3950 6100
Wire Wire Line
	3950 6100 3950 2400
Wire Wire Line
	3950 2400 3050 2400
Wire Wire Line
	3050 2200 4000 2200
Wire Wire Line
	4000 2200 4000 6150
Wire Wire Line
	4000 6150 3300 6150
Wire Wire Line
	3300 6150 3300 5900
Wire Wire Line
	3200 5900 3200 5950
Wire Wire Line
	5100 5950 5000 5950
Wire Wire Line
	5000 5950 5000 5900
Wire Wire Line
	4900 5900 4900 6000
Wire Wire Line
	4900 6000 5150 6000
Wire Wire Line
	5200 6050 4800 6050
Wire Wire Line
	4800 6050 4800 5900
Wire Wire Line
	4700 5900 4700 6100
Wire Wire Line
	4700 6100 5250 6100
Wire Wire Line
	5300 6150 4600 6150
Wire Wire Line
	4600 6150 4600 5900
Wire Wire Line
	4500 5900 4500 5950
Wire Wire Line
	6400 5950 6300 5950
Wire Wire Line
	6300 5950 6300 5900
Wire Wire Line
	6200 5900 6200 6000
Wire Wire Line
	6200 6000 6450 6000
Wire Wire Line
	6500 6050 6100 6050
Wire Wire Line
	6100 6050 6100 5900
Wire Wire Line
	6000 5900 6000 6100
Wire Wire Line
	6000 6100 6550 6100
Wire Wire Line
	6600 6150 5900 6150
Wire Wire Line
	5900 6150 5900 5900
Wire Wire Line
	5800 5900 5800 5950
Wire Wire Line
	4600 3100 4600 5400
Wire Wire Line
	3050 3100 4600 3100
Wire Wire Line
	3050 3300 4700 3300
Wire Wire Line
	4700 3300 4700 5400
Wire Wire Line
	4800 3500 4800 5400
Wire Wire Line
	3050 3500 4800 3500
Wire Wire Line
	3050 3700 4900 3700
Wire Wire Line
	4900 3700 4900 5400
Wire Wire Line
	3050 3900 5000 3900
Wire Wire Line
	5000 3900 5000 5400
Wire Wire Line
	5100 4000 5100 5950
Wire Wire Line
	3050 4000 5100 4000
Wire Wire Line
	3050 3800 5150 3800
Wire Wire Line
	5150 3800 5150 6000
Wire Wire Line
	5200 3600 5200 6050
Wire Wire Line
	3050 3600 5200 3600
Wire Wire Line
	3050 3400 5250 3400
Wire Wire Line
	5250 3400 5250 6100
Wire Wire Line
	5300 3200 5300 6150
Wire Wire Line
	3050 3200 5300 3200
Wire Wire Line
	5900 4100 5900 5400
Wire Wire Line
	3050 4100 5900 4100
Wire Wire Line
	3050 4300 6000 4300
Wire Wire Line
	6000 4300 6000 5400
Wire Wire Line
	6100 4500 6100 5400
Wire Wire Line
	3050 4500 6100 4500
Wire Wire Line
	3050 4700 6200 4700
Wire Wire Line
	6200 4700 6200 5400
Wire Wire Line
	6300 4900 6300 5400
Wire Wire Line
	3050 4900 6300 4900
Wire Wire Line
	6400 5000 6400 5950
Wire Wire Line
	3050 5000 6400 5000
Wire Wire Line
	6450 6000 6450 4800
Wire Wire Line
	6450 4800 3050 4800
Wire Wire Line
	6500 4600 6500 6050
Wire Wire Line
	3050 4600 6500 4600
Wire Wire Line
	6550 6100 6550 4400
Wire Wire Line
	6550 4400 3050 4400
Wire Wire Line
	6600 4200 6600 6150
Wire Wire Line
	3050 4200 6600 4200
Wire Wire Line
	6100 750  6100 850 
Wire Wire Line
	6100 800  6150 800 
Connection ~ 6100 800 
Wire Wire Line
	6100 1450 6100 1600
Wire Wire Line
	5900 1250 5850 1250
Wire Wire Line
	5850 1250 5850 1500
Wire Wire Line
	5850 1500 6550 1500
Connection ~ 6100 1550
Wire Wire Line
	5050 1050 5900 1050
Wire Wire Line
	2000 1250 1950 1250
Wire Wire Line
	1950 1250 1950 1500
Wire Wire Line
	1950 1500 2650 1500
Wire Wire Line
	1150 1050 2000 1050
Wire Wire Line
	7400 5300 7400 5400
Wire Wire Line
	6950 2000 7200 2000
Wire Wire Line
	3200 5300 3200 5400
Wire Wire Line
	4400 5100 4400 5400
Wire Wire Line
	4500 5300 4500 5400
Wire Wire Line
	5800 5300 5800 5400
Wire Wire Line
	7200 6200 7200 5900
Wire Wire Line
	7300 6250 7300 5900
Wire Wire Line
	2800 6200 7200 6200
Wire Wire Line
	4300 6200 4300 5900
Wire Wire Line
	5600 6200 5600 5900
Connection ~ 4300 6200
Wire Wire Line
	2800 6050 2800 6200
Connection ~ 3000 6200
Wire Wire Line
	2600 6050 2600 6250
Wire Wire Line
	2600 6250 7300 6250
Wire Wire Line
	3100 6250 3100 5900
Wire Wire Line
	4400 6250 4400 5900
Connection ~ 3100 6250
Wire Wire Line
	5700 6250 5700 5900
Connection ~ 4400 6250
Connection ~ 5600 6200
Connection ~ 5700 6250
Wire Wire Line
	3000 5900 3000 6200
Wire Wire Line
	3000 5400 3000 5350
Wire Wire Line
	2950 5350 5600 5350
Wire Wire Line
	4300 2000 4300 5400
Wire Wire Line
	5600 5350 5600 5400
Connection ~ 4300 5350
Wire Wire Line
	3050 2000 4300 2000
Connection ~ 3000 5350
Wire Wire Line
	2650 5350 2600 5350
Connection ~ 2600 5100
Wire Wire Line
	2200 1450 2200 1600
Connection ~ 2200 1550
Wire Wire Line
	2600 1150 3300 1150
Wire Wire Line
	2650 1500 2650 1150
Wire Wire Line
	1500 1550 2200 1550
Wire Wire Line
	2650 1900 2750 1900
Wire Wire Line
	2650 2000 2750 2000
Wire Wire Line
	2650 2100 2750 2100
Wire Wire Line
	2650 2200 2750 2200
Wire Wire Line
	2650 2300 2750 2300
Wire Wire Line
	2650 2400 2750 2400
Wire Wire Line
	2650 2500 2750 2500
Wire Wire Line
	2650 2600 2750 2600
Wire Wire Line
	2650 2700 2750 2700
Wire Wire Line
	2650 2800 2750 2800
Wire Wire Line
	2650 2900 2750 2900
Wire Wire Line
	2650 3000 2750 3000
Wire Wire Line
	2650 3100 2750 3100
Wire Wire Line
	2650 3200 2750 3200
Wire Wire Line
	2650 3300 2750 3300
Wire Wire Line
	2650 3400 2750 3400
Wire Wire Line
	2650 3500 2750 3500
Wire Wire Line
	2650 3600 2750 3600
Wire Wire Line
	2650 3700 2750 3700
Wire Wire Line
	2650 3800 2750 3800
Wire Wire Line
	2650 3900 2750 3900
Wire Wire Line
	2650 4000 2750 4000
Wire Wire Line
	2650 4100 2750 4100
Wire Wire Line
	2650 4200 2750 4200
Wire Wire Line
	2650 4300 2750 4300
Wire Wire Line
	2650 4400 2750 4400
Wire Wire Line
	2650 4500 2750 4500
Wire Wire Line
	2650 4600 2750 4600
Wire Wire Line
	2650 4700 2750 4700
Wire Wire Line
	2650 4800 2750 4800
Wire Wire Line
	2650 4900 2750 4900
Wire Wire Line
	2700 1600 2700 1900
Connection ~ 2700 1900
Connection ~ 2650 1150
Wire Wire Line
	2700 1600 2750 1600
Wire Wire Line
	3050 1600 3100 1600
Wire Wire Line
	6550 1900 6650 1900
Wire Wire Line
	6550 2000 6650 2000
Wire Wire Line
	6550 2100 6650 2100
Wire Wire Line
	6550 2200 6650 2200
Wire Wire Line
	6550 2300 6650 2300
Wire Wire Line
	6550 2400 6650 2400
Wire Wire Line
	6550 2500 6650 2500
Wire Wire Line
	6550 2600 6650 2600
Wire Wire Line
	6550 2700 6650 2700
Wire Wire Line
	6550 2800 6650 2800
Wire Wire Line
	6550 2900 6650 2900
Wire Wire Line
	6550 3000 6650 3000
Wire Wire Line
	6650 1600 6600 1600
Wire Wire Line
	6600 1600 6600 1900
Connection ~ 6600 1900
Wire Wire Line
	6950 1600 7000 1600
Wire Wire Line
	6500 1150 7200 1150
Wire Wire Line
	6550 1500 6550 1150
Connection ~ 6550 1150
Wire Wire Line
	5350 1550 6100 1550
Wire Wire Line
	6450 800  6500 800 
Wire Wire Line
	6500 800  6500 850 
Wire Wire Line
	7150 5350 7200 5350
Wire Wire Line
	7200 2000 7200 5400
Wire Wire Line
	6800 5350 6850 5350
Wire Wire Line
	6800 5100 7300 5100
Wire Wire Line
	7300 5100 7300 5400
Wire Wire Line
	5300 3100 6800 3100
Connection ~ 6800 5100
Connection ~ 7200 5350
$Comp
L R_ARRAY_4 R602
U 1 1 5874B970
P 2900 1900
F 0 "R602" V 2850 2100 60  0000 C CNN
F 1 "22" V 2900 1900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 1900 60  0001 C CNN
F 3 "" H 2900 1900 60  0001 C CNN
	1    2900 1900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R602
U 2 1 5874BD03
P 2900 2000
F 0 "R602" V 2850 2200 60  0000 C CNN
F 1 "22" V 2900 2000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2000 60  0001 C CNN
F 3 "" H 2900 2000 60  0001 C CNN
	2    2900 2000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R602
U 3 1 5874BD5E
P 2900 2100
F 0 "R602" V 2850 2300 60  0000 C CNN
F 1 "22" V 2900 2100 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2100 60  0001 C CNN
F 3 "" H 2900 2100 60  0001 C CNN
	3    2900 2100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R602
U 4 1 5874BDBC
P 2900 2200
F 0 "R602" V 2850 2400 60  0000 C CNN
F 1 "22" V 2900 2200 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2200 60  0001 C CNN
F 3 "" H 2900 2200 60  0001 C CNN
	4    2900 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 1600 7000 1650
Wire Wire Line
	6800 3100 6800 5350
Wire Wire Line
	2600 5350 2600 5100
Wire Wire Line
	3100 1600 3100 1650
$Comp
L R_ARRAY_4 R610
U 1 1 587507D0
P 5050 1300
F 0 "R610" V 4950 1300 60  0000 C CNN
F 1 "10k" V 5050 1300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 5050 1300 60  0001 C CNN
F 3 "" H 5050 1300 60  0001 C CNN
	1    5050 1300
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R610
U 2 1 58750B4C
P 5350 1300
F 0 "R610" V 5250 1300 60  0000 C CNN
F 1 "10k" V 5350 1300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 5350 1300 60  0001 C CNN
F 3 "" H 5350 1300 60  0001 C CNN
	2    5350 1300
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R507
U 3 1 58750C3F
P 1150 1300
F 0 "R507" V 1050 1300 60  0000 C CNN
F 1 "10k" V 1150 1300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 1150 1300 60  0001 C CNN
F 3 "" H 1150 1300 60  0001 C CNN
	3    1150 1300
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R507
U 4 1 58750EDC
P 1500 1300
F 0 "R507" V 1400 1300 60  0000 C CNN
F 1 "10k" V 1500 1300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 1500 1300 60  0001 C CNN
F 3 "" H 1500 1300 60  0001 C CNN
	4    1500 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1900 850  1950
Wire Wire Line
	1150 1450 1150 1950
Connection ~ 1150 1900
Wire Wire Line
	850  2250 850  2300
Wire Wire Line
	1150 2300 1150 2250
Wire Wire Line
	1000 2350 1000 2300
Connection ~ 1000 2300
$Comp
L C C604
U 1 1 587B510A
P 2900 1600
F 0 "C604" H 2925 1700 50  0000 L CNN
F 1 "4.7µF" H 2925 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 2938 1450 50  0001 C CNN
F 3 "" H 2900 1600 50  0000 C CNN
	1    2900 1600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR088
U 1 1 587B5285
P 3100 1650
F 0 "#PWR088" H 3100 1400 50  0001 C CNN
F 1 "GND" H 3100 1500 50  0001 C CNN
F 2 "" H 3100 1650 50  0000 C CNN
F 3 "" H 3100 1650 50  0000 C CNN
	1    3100 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR089
U 1 1 587B539D
P 2200 1600
F 0 "#PWR089" H 2200 1350 50  0001 C CNN
F 1 "GND" H 2200 1450 50  0001 C CNN
F 2 "" H 2200 1600 50  0000 C CNN
F 3 "" H 2200 1600 50  0000 C CNN
	1    2200 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1150 1150 1050
Wire Wire Line
	1700 1450 1700 1550
Connection ~ 1700 1550
Wire Wire Line
	1700 1150 1700 1050
Connection ~ 1700 1050
Wire Wire Line
	1500 1450 1500 1550
Wire Wire Line
	1500 1150 1500 1050
Connection ~ 1500 1050
$Comp
L C C607
U 1 1 586990A7
P 5550 1300
F 0 "C607" H 5575 1400 50  0000 L CNN
F 1 "100nF" H 5575 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5588 1150 50  0001 C CNN
F 3 "" H 5550 1300 50  0000 C CNN
	1    5550 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1050 5050 1150
Wire Wire Line
	5350 1150 5350 1050
Connection ~ 5350 1050
Wire Wire Line
	5550 1150 5550 1050
Connection ~ 5550 1050
Wire Wire Line
	1400 1900 1400 5100
Wire Wire Line
	5300 1900 5300 3100
Wire Wire Line
	5350 1550 5350 1450
Wire Wire Line
	5550 1450 5550 1550
Connection ~ 5550 1550
$Comp
L GND #PWR090
U 1 1 587B9847
P 6100 1600
F 0 "#PWR090" H 6100 1350 50  0001 C CNN
F 1 "GND" H 6100 1450 50  0001 C CNN
F 2 "" H 6100 1600 50  0000 C CNN
F 3 "" H 6100 1600 50  0000 C CNN
	1    6100 1600
	1    0    0    -1  
$EndComp
$Comp
L C C608
U 1 1 587B995D
P 6300 800
F 0 "C608" H 6325 900 50  0000 L CNN
F 1 "100nF" H 6325 700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6338 650 50  0001 C CNN
F 3 "" H 6300 800 50  0000 C CNN
	1    6300 800 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR091
U 1 1 587B9AE0
P 6500 850
F 0 "#PWR091" H 6500 600 50  0001 C CNN
F 1 "GND" H 6500 700 50  0001 C CNN
F 2 "" H 6500 850 50  0000 C CNN
F 3 "" H 6500 850 50  0000 C CNN
	1    6500 850 
	1    0    0    -1  
$EndComp
$Comp
L C C609
U 1 1 587B9BF6
P 6800 1600
F 0 "C609" H 6825 1700 50  0000 L CNN
F 1 "4.7µF" H 6825 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6838 1450 50  0001 C CNN
F 3 "" H 6800 1600 50  0000 C CNN
	1    6800 1600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR092
U 1 1 587B9DA5
P 7000 1650
F 0 "#PWR092" H 7000 1400 50  0001 C CNN
F 1 "GND" H 7000 1500 50  0001 C CNN
F 2 "" H 7000 1650 50  0000 C CNN
F 3 "" H 7000 1650 50  0000 C CNN
	1    7000 1650
	1    0    0    -1  
$EndComp
$Comp
L C C606
U 1 1 587B9F3F
P 5050 2100
F 0 "C606" H 5075 2200 50  0000 L CNN
F 1 "4.7µF" H 5075 2000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 5088 1950 50  0001 C CNN
F 3 "" H 5050 2100 50  0000 C CNN
	1    5050 2100
	1    0    0    -1  
$EndComp
$Comp
L C C605
U 1 1 587BA164
P 4750 2100
F 0 "C605" H 4775 2200 50  0000 L CNN
F 1 "4.7µF" H 4775 2000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4788 1950 50  0001 C CNN
F 3 "" H 4750 2100 50  0000 C CNN
	1    4750 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR093
U 1 1 587BA589
P 4900 2350
F 0 "#PWR093" H 4900 2100 50  0001 C CNN
F 1 "GND" H 4900 2200 50  0001 C CNN
F 2 "" H 4900 2350 50  0000 C CNN
F 3 "" H 4900 2350 50  0000 C CNN
	1    4900 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2250 4750 2300
Wire Wire Line
	4750 2300 5050 2300
Wire Wire Line
	5050 2300 5050 2250
Wire Wire Line
	4900 2300 4900 2350
Connection ~ 4900 2300
$Comp
L R_ARRAY_4 R603
U 1 1 587BF908
P 2900 2300
F 0 "R603" V 2850 2500 60  0000 C CNN
F 1 "22" V 2900 2300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2300 60  0001 C CNN
F 3 "" H 2900 2300 60  0001 C CNN
	1    2900 2300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R603
U 2 1 587BF90E
P 2900 2400
F 0 "R603" V 2850 2600 60  0000 C CNN
F 1 "22" V 2900 2400 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2400 60  0001 C CNN
F 3 "" H 2900 2400 60  0001 C CNN
	2    2900 2400
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R603
U 3 1 587BF914
P 2900 2500
F 0 "R603" V 2850 2700 60  0000 C CNN
F 1 "22" V 2900 2500 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2500 60  0001 C CNN
F 3 "" H 2900 2500 60  0001 C CNN
	3    2900 2500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R603
U 4 1 587BF91A
P 2900 2600
F 0 "R603" V 2850 2800 60  0000 C CNN
F 1 "22" V 2900 2600 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2600 60  0001 C CNN
F 3 "" H 2900 2600 60  0001 C CNN
	4    2900 2600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R604
U 1 1 587BFC96
P 2900 2700
F 0 "R604" V 2850 2900 60  0000 C CNN
F 1 "22" V 2900 2700 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2700 60  0001 C CNN
F 3 "" H 2900 2700 60  0001 C CNN
	1    2900 2700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R604
U 2 1 587BFC9C
P 2900 2800
F 0 "R604" V 2850 3000 60  0000 C CNN
F 1 "22" V 2900 2800 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2800 60  0001 C CNN
F 3 "" H 2900 2800 60  0001 C CNN
	2    2900 2800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R604
U 3 1 587BFCA2
P 2900 2900
F 0 "R604" V 2850 3100 60  0000 C CNN
F 1 "22" V 2900 2900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 2900 60  0001 C CNN
F 3 "" H 2900 2900 60  0001 C CNN
	3    2900 2900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R604
U 4 1 587BFCA8
P 2900 3000
F 0 "R604" V 2850 3200 60  0000 C CNN
F 1 "22" V 2900 3000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3000 60  0001 C CNN
F 3 "" H 2900 3000 60  0001 C CNN
	4    2900 3000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R605
U 1 1 587BFCAE
P 2900 3100
F 0 "R605" V 2850 3300 60  0000 C CNN
F 1 "22" V 2900 3100 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3100 60  0001 C CNN
F 3 "" H 2900 3100 60  0001 C CNN
	1    2900 3100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R605
U 2 1 587BFCB4
P 2900 3200
F 0 "R605" V 2850 3400 60  0000 C CNN
F 1 "22" V 2900 3200 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3200 60  0001 C CNN
F 3 "" H 2900 3200 60  0001 C CNN
	2    2900 3200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R605
U 3 1 587BFCBA
P 2900 3300
F 0 "R605" V 2850 3500 60  0000 C CNN
F 1 "22" V 2900 3300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3300 60  0001 C CNN
F 3 "" H 2900 3300 60  0001 C CNN
	3    2900 3300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R605
U 4 1 587BFCC0
P 2900 3400
F 0 "R605" V 2850 3600 60  0000 C CNN
F 1 "22" V 2900 3400 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3400 60  0001 C CNN
F 3 "" H 2900 3400 60  0001 C CNN
	4    2900 3400
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R606
U 1 1 587C0400
P 2900 3500
F 0 "R606" V 2850 3700 60  0000 C CNN
F 1 "22" V 2900 3500 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3500 60  0001 C CNN
F 3 "" H 2900 3500 60  0001 C CNN
	1    2900 3500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R606
U 2 1 587C0406
P 2900 3600
F 0 "R606" V 2850 3800 60  0000 C CNN
F 1 "22" V 2900 3600 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3600 60  0001 C CNN
F 3 "" H 2900 3600 60  0001 C CNN
	2    2900 3600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R606
U 3 1 587C040C
P 2900 3700
F 0 "R606" V 2850 3900 60  0000 C CNN
F 1 "22" V 2900 3700 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3700 60  0001 C CNN
F 3 "" H 2900 3700 60  0001 C CNN
	3    2900 3700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R606
U 4 1 587C0412
P 2900 3800
F 0 "R606" V 2850 4000 60  0000 C CNN
F 1 "22" V 2900 3800 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3800 60  0001 C CNN
F 3 "" H 2900 3800 60  0001 C CNN
	4    2900 3800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R607
U 1 1 587C0418
P 2900 3900
F 0 "R607" V 2850 4100 60  0000 C CNN
F 1 "22" V 2900 3900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 3900 60  0001 C CNN
F 3 "" H 2900 3900 60  0001 C CNN
	1    2900 3900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R607
U 2 1 587C041E
P 2900 4000
F 0 "R607" V 2850 4200 60  0000 C CNN
F 1 "22" V 2900 4000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4000 60  0001 C CNN
F 3 "" H 2900 4000 60  0001 C CNN
	2    2900 4000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R607
U 3 1 587C0424
P 2900 4100
F 0 "R607" V 2850 4300 60  0000 C CNN
F 1 "22" V 2900 4100 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4100 60  0001 C CNN
F 3 "" H 2900 4100 60  0001 C CNN
	3    2900 4100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R607
U 4 1 587C042A
P 2900 4200
F 0 "R607" V 2850 4400 60  0000 C CNN
F 1 "22" V 2900 4200 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4200 60  0001 C CNN
F 3 "" H 2900 4200 60  0001 C CNN
	4    2900 4200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R608
U 1 1 587C0430
P 2900 4300
F 0 "R608" V 2850 4500 60  0000 C CNN
F 1 "22" V 2900 4300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4300 60  0001 C CNN
F 3 "" H 2900 4300 60  0001 C CNN
	1    2900 4300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R608
U 2 1 587C0436
P 2900 4400
F 0 "R608" V 2850 4600 60  0000 C CNN
F 1 "22" V 2900 4400 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4400 60  0001 C CNN
F 3 "" H 2900 4400 60  0001 C CNN
	2    2900 4400
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R608
U 3 1 587C043C
P 2900 4500
F 0 "R608" V 2850 4700 60  0000 C CNN
F 1 "22" V 2900 4500 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4500 60  0001 C CNN
F 3 "" H 2900 4500 60  0001 C CNN
	3    2900 4500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R608
U 4 1 587C0442
P 2900 4600
F 0 "R608" V 2850 4800 60  0000 C CNN
F 1 "22" V 2900 4600 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4600 60  0001 C CNN
F 3 "" H 2900 4600 60  0001 C CNN
	4    2900 4600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R609
U 1 1 587C0448
P 2900 4700
F 0 "R609" V 2850 4900 60  0000 C CNN
F 1 "22" V 2900 4700 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4700 60  0001 C CNN
F 3 "" H 2900 4700 60  0001 C CNN
	1    2900 4700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R609
U 2 1 587C044E
P 2900 4800
F 0 "R609" V 2850 5000 60  0000 C CNN
F 1 "22" V 2900 4800 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4800 60  0001 C CNN
F 3 "" H 2900 4800 60  0001 C CNN
	2    2900 4800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R609
U 3 1 587C0454
P 2900 4900
F 0 "R609" V 2850 5100 60  0000 C CNN
F 1 "22" V 2900 4900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 4900 60  0001 C CNN
F 3 "" H 2900 4900 60  0001 C CNN
	3    2900 4900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R609
U 4 1 587C045A
P 2900 5000
F 0 "R609" V 2850 5200 60  0000 C CNN
F 1 "22" V 2900 5000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 2900 5000 60  0001 C CNN
F 3 "" H 2900 5000 60  0001 C CNN
	4    2900 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 5000 2650 5000
$Comp
L R R614
U 1 1 587C4766
P 7000 5350
F 0 "R614" V 6900 5350 50  0000 C CNN
F 1 "6.8k" V 7000 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6930 5350 50  0001 C CNN
F 3 "" H 7000 5350 50  0000 C CNN
	1    7000 5350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR094
U 1 1 587C4D4E
P 3200 5950
F 0 "#PWR094" H 3200 5700 50  0001 C CNN
F 1 "GND" H 3200 5800 50  0001 C CNN
F 2 "" H 3200 5950 50  0000 C CNN
F 3 "" H 3200 5950 50  0000 C CNN
	1    3200 5950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR095
U 1 1 587C5154
P 4500 5950
F 0 "#PWR095" H 4500 5700 50  0001 C CNN
F 1 "GND" H 4500 5800 50  0001 C CNN
F 2 "" H 4500 5950 50  0000 C CNN
F 3 "" H 4500 5950 50  0000 C CNN
	1    4500 5950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR096
U 1 1 587C5228
P 5800 5950
F 0 "#PWR096" H 5800 5700 50  0001 C CNN
F 1 "GND" H 5800 5800 50  0001 C CNN
F 2 "" H 5800 5950 50  0000 C CNN
F 3 "" H 5800 5950 50  0000 C CNN
	1    5800 5950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR097
U 1 1 587C52FC
P 7400 5950
F 0 "#PWR097" H 7400 5700 50  0001 C CNN
F 1 "GND" H 7400 5800 50  0001 C CNN
F 2 "" H 7400 5950 50  0000 C CNN
F 3 "" H 7400 5950 50  0000 C CNN
	1    7400 5950
	1    0    0    -1  
$EndComp
$Comp
L R_ARRAY_4 R611
U 1 1 587C6525
P 6800 1900
F 0 "R611" V 6750 2100 60  0000 C CNN
F 1 "22" V 6800 1900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 1900 60  0001 C CNN
F 3 "" H 6800 1900 60  0001 C CNN
	1    6800 1900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R611
U 2 1 587C652B
P 6800 2000
F 0 "R611" V 6750 2200 60  0000 C CNN
F 1 "22" V 6800 2000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2000 60  0001 C CNN
F 3 "" H 6800 2000 60  0001 C CNN
	2    6800 2000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R611
U 3 1 587C6531
P 6800 2100
F 0 "R611" V 6750 2300 60  0000 C CNN
F 1 "22" V 6800 2100 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2100 60  0001 C CNN
F 3 "" H 6800 2100 60  0001 C CNN
	3    6800 2100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R611
U 4 1 587C6537
P 6800 2200
F 0 "R611" V 6750 2400 60  0000 C CNN
F 1 "22" V 6800 2200 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2200 60  0001 C CNN
F 3 "" H 6800 2200 60  0001 C CNN
	4    6800 2200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R612
U 1 1 587C653D
P 6800 2300
F 0 "R612" V 6750 2500 60  0000 C CNN
F 1 "22" V 6800 2300 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2300 60  0001 C CNN
F 3 "" H 6800 2300 60  0001 C CNN
	1    6800 2300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R612
U 2 1 587C6543
P 6800 2400
F 0 "R612" V 6750 2600 60  0000 C CNN
F 1 "22" V 6800 2400 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2400 60  0001 C CNN
F 3 "" H 6800 2400 60  0001 C CNN
	2    6800 2400
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R612
U 3 1 587C6549
P 6800 2500
F 0 "R612" V 6750 2700 60  0000 C CNN
F 1 "22" V 6800 2500 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2500 60  0001 C CNN
F 3 "" H 6800 2500 60  0001 C CNN
	3    6800 2500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R612
U 4 1 587C654F
P 6800 2600
F 0 "R612" V 6750 2800 60  0000 C CNN
F 1 "22" V 6800 2600 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2600 60  0001 C CNN
F 3 "" H 6800 2600 60  0001 C CNN
	4    6800 2600
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R613
U 1 1 587C679F
P 6800 2700
F 0 "R613" V 6750 2900 60  0000 C CNN
F 1 "22" V 6800 2700 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2700 60  0001 C CNN
F 3 "" H 6800 2700 60  0001 C CNN
	1    6800 2700
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R613
U 2 1 587C67A5
P 6800 2800
F 0 "R613" V 6750 3000 60  0000 C CNN
F 1 "22" V 6800 2800 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2800 60  0001 C CNN
F 3 "" H 6800 2800 60  0001 C CNN
	2    6800 2800
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R613
U 3 1 587C67AB
P 6800 2900
F 0 "R613" V 6750 3100 60  0000 C CNN
F 1 "22" V 6800 2900 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 2900 60  0001 C CNN
F 3 "" H 6800 2900 60  0001 C CNN
	3    6800 2900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R613
U 4 1 587C67B1
P 6800 3000
F 0 "R613" V 6750 3200 60  0000 C CNN
F 1 "22" V 6800 3000 60  0000 C CNN
F 2 "Resistors_SMD:R_Array_Convex_4x0603" H 6800 3000 60  0001 C CNN
F 3 "" H 6800 3000 60  0001 C CNN
	4    6800 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 1900 7200 1900
Wire Wire Line
	7200 1900 7200 1150
Wire Wire Line
	3050 1900 3300 1900
Wire Wire Line
	3300 1900 3300 1150
$EndSCHEMATC
