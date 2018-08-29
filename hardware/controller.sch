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
Sheet 3 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_NPN_CBE Q?
U 1 1 5AE2120C
P 7050 2300
F 0 "Q?" H 7250 2350 50  0000 L CNN
F 1 "Q_NPN_CBE" H 7250 2250 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 7250 2400 50  0001 C CNN
F 3 "" H 7050 2300 50  0001 C CNN
	1    7050 2300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AE21286
P 7150 2700
F 0 "R?" V 7230 2700 50  0000 C CNN
F 1 "10k" V 7150 2700 50  0000 C CNN
F 2 "" V 7080 2700 50  0001 C CNN
F 3 "" H 7150 2700 50  0001 C CNN
	1    7150 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5AE212EB
P 7150 2900
F 0 "#PWR01" H 7150 2650 50  0001 C CNN
F 1 "GND" H 7000 2900 50  0000 C CNN
F 2 "" H 7150 2900 50  0001 C CNN
F 3 "" H 7150 2900 50  0001 C CNN
	1    7150 2900
	1    0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5AE2130F
P 6400 2050
F 0 "D?" H 6400 2150 50  0000 C CNN
F 1 "D" H 6400 1950 50  0000 C CNN
F 2 "" H 6400 2050 50  0001 C CNN
F 3 "" H 6400 2050 50  0001 C CNN
	1    6400 2050
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5AE2133D
P 6800 2100
F 0 "R?" V 6880 2100 50  0000 C CNN
F 1 "10k" V 6800 2100 50  0000 C CNN
F 2 "" V 6730 2100 50  0001 C CNN
F 3 "" H 6800 2100 50  0001 C CNN
	1    6800 2100
	1    0    0    -1  
$EndComp
$Comp
L LM393 U?
U 2 1 5B075203
P 6050 2300
F 0 "U?" H 6200 2450 50  0000 C CNN
F 1 "LM393" H 6300 2150 50  0000 C CNN
F 2 "" H 6050 2300 50  0001 C CNN
F 3 "" H 6050 2300 50  0001 C CNN
	2    6050 2300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B07539F
P 6200 1750
F 0 "R?" V 6280 1750 50  0000 C CNN
F 1 "1M" V 6200 1750 50  0000 C CNN
F 2 "" V 6130 1750 50  0001 C CNN
F 3 "" H 6200 1750 50  0001 C CNN
	1    6200 1750
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5B0755A4
P 2600 900
F 0 "C?" H 2625 1000 50  0000 L CNN
F 1 "100nF" H 2625 800 50  0000 L CNN
F 2 "" H 2638 750 50  0001 C CNN
F 3 "" H 2600 900 50  0001 C CNN
	1    2600 900 
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5B07603E
P 2400 900
F 0 "C?" H 2425 1000 50  0000 L CNN
F 1 "CP" H 2425 800 50  0000 L CNN
F 2 "" H 2438 750 50  0001 C CNN
F 3 "" H 2400 900 50  0001 C CNN
	1    2400 900 
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B0764EB
P 5500 2400
F 0 "R?" V 5580 2400 50  0000 C CNN
F 1 "1k" V 5500 2400 50  0000 C CNN
F 2 "" V 5430 2400 50  0001 C CNN
F 3 "" H 5500 2400 50  0001 C CNN
	1    5500 2400
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5B076A4E
P 5000 2900
F 0 "C?" H 5025 3000 50  0000 L CNN
F 1 "1nF" H 5025 2800 50  0000 L CNN
F 2 "" H 5038 2750 50  0001 C CNN
F 3 "" H 5000 2900 50  0001 C CNN
	1    5000 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5B076C12
P 4250 3250
F 0 "#PWR03" H 4250 3000 50  0001 C CNN
F 1 "GND" H 4250 3100 50  0001 C CNN
F 2 "" H 4250 3250 50  0001 C CNN
F 3 "" H 4250 3250 50  0001 C CNN
	1    4250 3250
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_CBE Q?
U 1 1 5B076EFD
P 4600 2650
F 0 "Q?" H 4800 2700 50  0000 L CNN
F 1 "Q_NPN_CBE" H 4800 2600 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4800 2750 50  0001 C CNN
F 3 "" H 4600 2650 50  0001 C CNN
	1    4600 2650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B077027
P 4700 3050
F 0 "R?" V 4780 3050 50  0000 C CNN
F 1 "470" V 4700 3050 50  0000 C CNN
F 2 "" V 4630 3050 50  0001 C CNN
F 3 "" H 4700 3050 50  0001 C CNN
	1    4700 3050
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B077158
P 3150 3050
F 0 "R?" V 3230 3050 50  0000 C CNN
F 1 "4.7k" V 3150 3050 50  0000 C CNN
F 2 "" V 3080 3050 50  0001 C CNN
F 3 "" H 3150 3050 50  0001 C CNN
	1    3150 3050
	-1   0    0    1   
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B0777C5
P 4600 2150
F 0 "Q?" H 4800 2200 50  0000 L CNN
F 1 "Q_PNP_CBE" H 4800 2100 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4800 2250 50  0001 C CNN
F 3 "" H 4600 2150 50  0001 C CNN
	1    4600 2150
	1    0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B0779AD
P 4250 3050
F 0 "R?" V 4330 3050 50  0000 C CNN
F 1 "47k" V 4250 3050 50  0000 C CNN
F 2 "" V 4180 3050 50  0001 C CNN
F 3 "" H 4250 3050 50  0001 C CNN
	1    4250 3050
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B077BED
P 4250 1750
F 0 "R?" V 4330 1750 50  0000 C CNN
F 1 "10k" V 4250 1750 50  0000 C CNN
F 2 "" V 4180 1750 50  0001 C CNN
F 3 "" H 4250 1750 50  0001 C CNN
	1    4250 1750
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B078796
P 4700 1750
F 0 "R?" V 4780 1750 50  0000 C CNN
F 1 "4.7k" V 4700 1750 50  0000 C CNN
F 2 "" V 4630 1750 50  0001 C CNN
F 3 "" H 4700 1750 50  0001 C CNN
	1    4700 1750
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 5B079424
P 2050 1150
F 0 "#PWR04" H 2050 900 50  0001 C CNN
F 1 "GND" H 2050 1000 50  0000 C CNN
F 2 "" H 2050 1150 50  0001 C CNN
F 3 "" H 2050 1150 50  0001 C CNN
	1    2050 1150
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B079AAE
P 3250 2400
F 0 "Q?" H 3450 2450 50  0000 L CNN
F 1 "Q_PNP_CBE" H 3450 2350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 3450 2500 50  0001 C CNN
F 3 "" H 3250 2400 50  0001 C CNN
	1    3250 2400
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B079CB4
P 3500 2200
F 0 "R?" V 3580 2200 50  0000 C CNN
F 1 "47k" V 3500 2200 50  0000 C CNN
F 2 "" V 3430 2200 50  0001 C CNN
F 3 "" H 3500 2200 50  0001 C CNN
	1    3500 2200
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B079F4E
P 3700 2850
F 0 "R?" V 3780 2850 50  0000 C CNN
F 1 "22k" V 3700 2850 50  0000 C CNN
F 2 "" V 3630 2850 50  0001 C CNN
F 3 "" H 3700 2850 50  0001 C CNN
	1    3700 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B07A258
P 3900 3050
F 0 "R?" V 3980 3050 50  0000 C CNN
F 1 "10k" V 3900 3050 50  0000 C CNN
F 2 "" V 3830 3050 50  0001 C CNN
F 3 "" H 3900 3050 50  0001 C CNN
	1    3900 3050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B07AAD9
P 3900 1750
F 0 "R?" V 3980 1750 50  0000 C CNN
F 1 "10k" V 3900 1750 50  0000 C CNN
F 2 "" V 3830 1750 50  0001 C CNN
F 3 "" H 3900 1750 50  0001 C CNN
	1    3900 1750
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B07BC12
P 4450 2400
F 0 "R?" V 4530 2400 50  0000 C CNN
F 1 "10k" V 4450 2400 50  0000 C CNN
F 2 "" V 4380 2400 50  0001 C CNN
F 3 "" H 4450 2400 50  0001 C CNN
F 4 "Must be small enough to have proper trigger levels" V 4450 2400 60  0001 C CNN "Hint"
	1    4450 2400
	0    1    1    0   
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B07A06E
P 4000 2400
F 0 "Q?" H 4200 2450 50  0000 L CNN
F 1 "Q_PNP_CBE" H 4200 2350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4200 2500 50  0001 C CNN
F 3 "" H 4000 2400 50  0001 C CNN
	1    4000 2400
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B08268D
P 7700 5000
F 0 "R?" V 7780 5000 50  0000 C CNN
F 1 "100k" V 7700 5000 50  0000 C CNN
F 2 "" V 7630 5000 50  0001 C CNN
F 3 "" H 7700 5000 50  0001 C CNN
	1    7700 5000
	0    -1   -1   0   
$EndComp
$Comp
L LM324 U?
U 1 1 5B08311B
P 3150 4350
F 0 "U?" H 3150 4550 50  0000 L CNN
F 1 "LM324" H 3150 4150 50  0000 L CNN
F 2 "" H 3100 4450 50  0001 C CNN
F 3 "" H 3200 4550 50  0001 C CNN
	1    3150 4350
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B083346
P 3700 4450
F 0 "R?" V 3780 4450 50  0000 C CNN
F 1 "4.7k" V 3700 4450 50  0000 C CNN
F 2 "" V 3630 4450 50  0001 C CNN
F 3 "" H 3700 4450 50  0001 C CNN
	1    3700 4450
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5B083893
P 3500 4650
F 0 "C?" H 3525 4750 50  0000 L CNN
F 1 "4.7nF" H 3525 4550 50  0000 L CNN
F 2 "" H 3538 4500 50  0001 C CNN
F 3 "" H 3500 4650 50  0001 C CNN
	1    3500 4650
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B083DAF
P 3150 4850
F 0 "R?" V 3230 4850 50  0000 C CNN
F 1 "10k" V 3150 4850 50  0000 C CNN
F 2 "" V 3080 4850 50  0001 C CNN
F 3 "" H 3150 4850 50  0001 C CNN
	1    3150 4850
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5B084110
P 3150 5050
F 0 "C?" H 3175 5150 50  0000 L CNN
F 1 "470pF" H 3175 4950 50  0000 L CNN
F 2 "" H 3188 4900 50  0001 C CNN
F 3 "" H 3150 5050 50  0001 C CNN
	1    3150 5050
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5B0852B5
P 1600 2850
F 0 "R?" V 1680 2850 50  0000 C CNN
F 1 "68k" V 1600 2850 50  0000 C CNN
F 2 "" V 1530 2850 50  0001 C CNN
F 3 "" H 1600 2850 50  0001 C CNN
	1    1600 2850
	0    1    1    0   
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B085450
P 2050 2850
F 0 "Q?" H 2250 2900 50  0000 L CNN
F 1 "Q_PNP_CBE" H 2250 2800 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 2250 2950 50  0001 C CNN
F 3 "" H 2050 2850 50  0001 C CNN
	1    2050 2850
	1    0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B08557B
P 1800 3050
F 0 "R?" V 1880 3050 50  0000 C CNN
F 1 "4.7k" V 1800 3050 50  0000 C CNN
F 2 "" V 1730 3050 50  0001 C CNN
F 3 "" H 1800 3050 50  0001 C CNN
	1    1800 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5B08570E
P 1800 3250
F 0 "#PWR08" H 1800 3000 50  0001 C CNN
F 1 "GND" H 1800 3100 50  0000 C CNN
F 2 "" H 1800 3250 50  0001 C CNN
F 3 "" H 1800 3250 50  0001 C CNN
	1    1800 3250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B085B6C
P 2150 2450
F 0 "R?" V 2230 2450 50  0000 C CNN
F 1 "4.7k" V 2150 2450 50  0000 C CNN
F 2 "" V 2080 2450 50  0001 C CNN
F 3 "" H 2150 2450 50  0001 C CNN
	1    2150 2450
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B085EAD
P 2050 2000
F 0 "Q?" H 2250 2050 50  0000 L CNN
F 1 "Q_PNP_CBE" H 2250 1950 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 2250 2100 50  0001 C CNN
F 3 "" H 2050 2000 50  0001 C CNN
	1    2050 2000
	1    0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B086288
P 2150 1600
F 0 "R?" V 2230 1600 50  0000 C CNN
F 1 "4.7k" V 2150 1600 50  0000 C CNN
F 2 "" V 2080 1600 50  0001 C CNN
F 3 "" H 2150 1600 50  0001 C CNN
	1    2150 1600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B0867A4
P 1650 1600
F 0 "R?" V 1730 1600 50  0000 C CNN
F 1 "22k" V 1650 1600 50  0000 C CNN
F 2 "" V 1580 1600 50  0001 C CNN
F 3 "" H 1650 1600 50  0001 C CNN
	1    1650 1600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B086CFD
P 1650 2200
F 0 "R?" V 1730 2200 50  0000 C CNN
F 1 "33k" V 1650 2200 50  0000 C CNN
F 2 "" V 1580 2200 50  0001 C CNN
F 3 "" H 1650 2200 50  0001 C CNN
	1    1650 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5B087101
P 1650 2400
F 0 "#PWR09" H 1650 2150 50  0001 C CNN
F 1 "GND" H 1650 2250 50  0000 C CNN
F 2 "" H 1650 2400 50  0001 C CNN
F 3 "" H 1650 2400 50  0001 C CNN
	1    1650 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5B087CCC
P 2150 3250
F 0 "#PWR010" H 2150 3000 50  0001 C CNN
F 1 "GND" H 2150 3100 50  0000 C CNN
F 2 "" H 2150 3250 50  0001 C CNN
F 3 "" H 2150 3250 50  0001 C CNN
	1    2150 3250
	1    0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B08B2DF
P 2800 4150
F 0 "D?" H 2800 4250 50  0000 C CNN
F 1 "D" H 2800 4050 50  0000 C CNN
F 2 "" H 2800 4150 50  0001 C CNN
F 3 "" H 2800 4150 50  0001 C CNN
	1    2800 4150
	0    -1   -1   0   
$EndComp
$Comp
L LM324 U?
U 1 1 5B092391
P 3900 5650
F 0 "U?" H 3900 5850 50  0000 L CNN
F 1 "LM324" H 3900 5450 50  0000 L CNN
F 2 "" H 3850 5750 50  0001 C CNN
F 3 "" H 3950 5850 50  0001 C CNN
	1    3900 5650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B092397
P 3550 5950
F 0 "C?" H 3575 6050 50  0000 L CNN
F 1 "560pF" H 3575 5850 50  0000 L CNN
F 2 "" H 3588 5800 50  0001 C CNN
F 3 "" H 3550 5950 50  0001 C CNN
	1    3550 5950
	1    0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B09239D
P 3900 6150
F 0 "R?" V 3980 6150 50  0000 C CNN
F 1 "47k" V 3900 6150 50  0000 C CNN
F 2 "" V 3830 6150 50  0001 C CNN
F 3 "" H 3900 6150 50  0001 C CNN
	1    3900 6150
	0    -1   1    0   
$EndComp
$Comp
L C C?
U 1 1 5B0923A4
P 3900 6350
F 0 "C?" H 3925 6450 50  0000 L CNN
F 1 "100pF" H 3925 6250 50  0000 L CNN
F 2 "" H 3938 6200 50  0001 C CNN
F 3 "" H 3900 6350 50  0001 C CNN
	1    3900 6350
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5B0927CC
P 3050 5750
F 0 "R?" V 3130 5750 50  0000 C CNN
F 1 "10k" V 3050 5750 50  0000 C CNN
F 2 "" V 2980 5750 50  0001 C CNN
F 3 "" H 3050 5750 50  0001 C CNN
	1    3050 5750
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5B092EFC
P 3250 5950
F 0 "R?" V 3330 5950 50  0000 C CNN
F 1 "1.3k" V 3250 5950 50  0000 C CNN
F 2 "" V 3180 5950 50  0001 C CNN
F 3 "" H 3250 5950 50  0001 C CNN
	1    3250 5950
	-1   0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B094382
P 5400 5850
F 0 "D?" H 5400 5950 50  0000 C CNN
F 1 "D" H 5400 5750 50  0000 C CNN
F 2 "" H 5400 5850 50  0001 C CNN
F 3 "" H 5400 5850 50  0001 C CNN
	1    5400 5850
	0    1    -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B095C9A
P 2650 6650
F 0 "R?" V 2730 6650 50  0000 C CNN
F 1 "10k" V 2650 6650 50  0000 C CNN
F 2 "" V 2580 6650 50  0001 C CNN
F 3 "" H 2650 6650 50  0001 C CNN
	1    2650 6650
	-1   0    0    1   
$EndComp
$Comp
L POT RV?
U 1 1 5B095DED
P 2650 7000
F 0 "RV?" V 2475 7000 50  0000 C CNN
F 1 "10k" V 2550 7000 50  0000 C CNN
F 2 "" H 2650 7000 50  0001 C CNN
F 3 "" H 2650 7000 50  0001 C CNN
	1    2650 7000
	1    0    0    -1  
$EndComp
$Comp
L LM324 U?
U 1 1 5B1452E0
P 6350 6050
F 0 "U?" H 6350 6250 50  0000 L CNN
F 1 "LM324" H 6350 5850 50  0000 L CNN
F 2 "" H 6300 6150 50  0001 C CNN
F 3 "" H 6400 6250 50  0001 C CNN
	1    6350 6050
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B1424A6
P 5600 4950
F 0 "R?" V 5680 4950 50  0000 C CNN
F 1 "10k" V 5600 4950 50  0000 C CNN
F 2 "" V 5530 4950 50  0001 C CNN
F 3 "" H 5600 4950 50  0001 C CNN
	1    5600 4950
	-1   0    0    1   
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B142A97
P 5500 5400
F 0 "Q?" H 5700 5450 50  0000 L CNN
F 1 "Q_PNP_CBE" H 5700 5350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 5700 5500 50  0001 C CNN
F 3 "" H 5500 5400 50  0001 C CNN
	1    5500 5400
	1    0    0    1   
$EndComp
$Comp
L POT RV?
U 1 1 5B1431A8
P 6000 5150
F 0 "RV?" V 5825 5150 50  0000 C CNN
F 1 "20k" V 5900 5150 50  0000 C CNN
F 2 "" H 6000 5150 50  0001 C CNN
F 3 "" H 6000 5150 50  0001 C CNN
	1    6000 5150
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B1434C5
P 6000 5500
F 0 "R?" V 6080 5500 50  0000 C CNN
F 1 "100k" V 6000 5500 50  0000 C CNN
F 2 "" V 5930 5500 50  0001 C CNN
F 3 "" H 6000 5500 50  0001 C CNN
	1    6000 5500
	1    0    0    -1  
$EndComp
$Comp
L LM324 U?
U 1 1 5B1457C5
P 5400 4250
F 0 "U?" H 5400 4450 50  0000 L CNN
F 1 "LM324" H 5400 4050 50  0000 L CNN
F 2 "" H 5350 4350 50  0001 C CNN
F 3 "" H 5450 4450 50  0001 C CNN
	1    5400 4250
	-1   0    0    -1  
$EndComp
$Comp
L Q_PNP_CBE Q?
U 1 1 5B146651
P 5000 5400
F 0 "Q?" H 5200 5450 50  0000 L CNN
F 1 "Q_PNP_CBE" H 5200 5350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 5200 5500 50  0001 C CNN
F 3 "" H 5000 5400 50  0001 C CNN
	1    5000 5400
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B146803
P 4900 4950
F 0 "R?" V 4980 4950 50  0000 C CNN
F 1 "10k" V 4900 4950 50  0000 C CNN
F 2 "" V 4830 4950 50  0001 C CNN
F 3 "" H 4900 4950 50  0001 C CNN
	1    4900 4950
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B14B8C1
P 5450 3800
F 0 "R?" V 5530 3800 50  0000 C CNN
F 1 "10k" V 5450 3800 50  0000 C CNN
F 2 "" V 5380 3800 50  0001 C CNN
F 3 "" H 5450 3800 50  0001 C CNN
	1    5450 3800
	0    -1   -1   0   
$EndComp
$Comp
L D D?
U 1 1 5B14FB2A
P 4900 4250
F 0 "D?" H 4900 4350 50  0000 C CNN
F 1 "D" H 4900 4150 50  0000 C CNN
F 2 "" H 4900 4250 50  0001 C CNN
F 3 "" H 4900 4250 50  0001 C CNN
	1    4900 4250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B151A22
P 3700 4250
F 0 "R?" V 3780 4250 50  0000 C CNN
F 1 "4.7k" V 3700 4250 50  0000 C CNN
F 2 "" V 3630 4250 50  0001 C CNN
F 3 "" H 3700 4250 50  0001 C CNN
	1    3700 4250
	0    1    1    0   
$EndComp
$Comp
L LM393 U?
U 1 1 5B16142E
P 4150 7100
F 0 "U?" H 4300 7250 50  0000 C CNN
F 1 "LM393" H 4400 6950 50  0000 C CNN
F 2 "" H 4150 7100 50  0001 C CNN
F 3 "" H 4150 7100 50  0001 C CNN
	1    4150 7100
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B161CD6
P 3050 7000
F 0 "R?" V 3130 7000 50  0000 C CNN
F 1 "5.6k" V 3050 7000 50  0000 C CNN
F 2 "" V 2980 7000 50  0001 C CNN
F 3 "" H 3050 7000 50  0001 C CNN
	1    3050 7000
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5B16325E
P 4600 6850
F 0 "R?" V 4680 6850 50  0000 C CNN
F 1 "1M" V 4600 6850 50  0000 C CNN
F 2 "" V 4530 6850 50  0001 C CNN
F 3 "" H 4600 6850 50  0001 C CNN
	1    4600 6850
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B16439D
P 4300 6750
F 0 "R?" V 4380 6750 50  0000 C CNN
F 1 "10k" V 4300 6750 50  0000 C CNN
F 2 "" V 4230 6750 50  0001 C CNN
F 3 "" H 4300 6750 50  0001 C CNN
	1    4300 6750
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5B1674FA
P 3250 7400
F 0 "R?" V 3330 7400 50  0000 C CNN
F 1 "220k" V 3250 7400 50  0000 C CNN
F 2 "" V 3180 7400 50  0001 C CNN
F 3 "" H 3250 7400 50  0001 C CNN
	1    3250 7400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B16D286
P 3600 7000
F 0 "R?" V 3680 7000 50  0000 C CNN
F 1 "47k" V 3600 7000 50  0000 C CNN
F 2 "" V 3530 7000 50  0001 C CNN
F 3 "" H 3600 7000 50  0001 C CNN
	1    3600 7000
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5B17001B
P 2650 7350
F 0 "D?" H 2650 7450 50  0000 C CNN
F 1 "D" H 2650 7250 50  0000 C CNN
F 2 "" H 2650 7350 50  0001 C CNN
F 3 "" H 2650 7350 50  0001 C CNN
	1    2650 7350
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B08FA97
P 4700 4550
F 0 "R?" V 4780 4550 50  0000 C CNN
F 1 "10k" V 4700 4550 50  0000 C CNN
F 2 "" V 4630 4550 50  0001 C CNN
F 3 "" H 4700 4550 50  0001 C CNN
	1    4700 4550
	1    0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B147345
P 5600 5850
F 0 "R?" V 5680 5850 50  0000 C CNN
F 1 "10k" V 5600 5850 50  0000 C CNN
F 2 "" V 5530 5850 50  0001 C CNN
F 3 "" H 5600 5850 50  0001 C CNN
	1    5600 5850
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5B091183
P 5800 4550
F 0 "R?" V 5880 4550 50  0000 C CNN
F 1 "10k" V 5800 4550 50  0000 C CNN
F 2 "" V 5730 4550 50  0001 C CNN
F 3 "" H 5800 4550 50  0001 C CNN
	1    5800 4550
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B16AC1D
P 3600 7200
F 0 "R?" V 3680 7200 50  0000 C CNN
F 1 "100k" V 3600 7200 50  0000 C CNN
F 2 "" V 3530 7200 50  0001 C CNN
F 3 "" H 3600 7200 50  0001 C CNN
	1    3600 7200
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5B3AD3B6
P 2850 7350
F 0 "C?" H 2875 7450 50  0000 L CNN
F 1 "100nF" H 2875 7250 50  0000 L CNN
F 2 "" H 2888 7200 50  0001 C CNN
F 3 "" H 2850 7350 50  0001 C CNN
	1    2850 7350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B3B5DB1
P 3800 7400
F 0 "C?" H 3825 7500 50  0000 L CNN
F 1 "100nF" H 3550 7300 50  0000 L CNN
F 2 "" H 3838 7250 50  0001 C CNN
F 3 "" H 3800 7400 50  0001 C CNN
	1    3800 7400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 5B3C7295
P 6000 5700
F 0 "#PWR015" H 6000 5450 50  0001 C CNN
F 1 "GND" H 6000 5550 50  0001 C CNN
F 2 "" H 6000 5700 50  0001 C CNN
F 3 "" H 6000 5700 50  0001 C CNN
	1    6000 5700
	1    0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B162930
P 4500 5650
F 0 "D?" H 4500 5750 50  0000 C CNN
F 1 "D" H 4500 5550 50  0000 C CNN
F 2 "" H 4500 5650 50  0001 C CNN
F 3 "" H 4500 5650 50  0001 C CNN
	1    4500 5650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B432965
P 2800 900
F 0 "C?" H 2825 1000 50  0000 L CNN
F 1 "100nF" H 2825 800 50  0000 L CNN
F 2 "" H 2838 750 50  0001 C CNN
F 3 "" H 2800 900 50  0001 C CNN
	1    2800 900 
	1    0    0    -1  
$EndComp
$Comp
L LP2950-5.0_TO92 U?
U 1 1 5B44BDB4
P 2050 700
F 0 "U?" H 1900 825 50  0000 C CNN
F 1 "LP2950L-5.0" H 2050 825 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 2050 925 50  0001 C CIN
F 3 "" H 2050 650 50  0001 C CNN
	1    2050 700 
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5B44D340
P 1700 900
F 0 "C?" H 1725 1000 50  0000 L CNN
F 1 "CP" H 1725 800 50  0000 L CNN
F 2 "" H 1738 750 50  0001 C CNN
F 3 "" H 1700 900 50  0001 C CNN
	1    1700 900 
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B464964
P 5500 2200
F 0 "R?" V 5580 2200 50  0000 C CNN
F 1 "1k" V 5500 2200 50  0000 C CNN
F 2 "" V 5430 2200 50  0001 C CNN
F 3 "" H 5500 2200 50  0001 C CNN
	1    5500 2200
	0    1    1    0   
$EndComp
Text Notes 3150 3450 0    60   ~ 0
Sawtooth 1.9V to 2.9V (Measured)
$Comp
L POT RV?
U 1 1 5B469253
P 1250 2850
F 0 "RV?" V 1075 2850 50  0000 C CNN
F 1 "20k" V 1150 2850 50  0000 C CNN
F 2 "" H 1250 2850 50  0001 C CNN
F 3 "" H 1250 2850 50  0001 C CNN
	1    1250 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B46C1CF
P 7100 5950
F 0 "R?" V 7180 5950 50  0000 C CNN
F 1 "100k" V 7100 5950 50  0000 C CNN
F 2 "" V 7030 5950 50  0001 C CNN
F 3 "" H 7100 5950 50  0001 C CNN
	1    7100 5950
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5B46C3DD
P 6900 6150
F 0 "C?" H 6925 6250 50  0000 L CNN
F 1 "100nF" H 6925 6050 50  0000 L CNN
F 2 "" H 6938 6000 50  0001 C CNN
F 3 "" H 6900 6150 50  0001 C CNN
	1    6900 6150
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B46D02C
P 6400 6450
F 0 "R?" V 6480 6450 50  0000 C CNN
F 1 "100k" V 6400 6450 50  0000 C CNN
F 2 "" V 6330 6450 50  0001 C CNN
F 3 "" H 6400 6450 50  0001 C CNN
	1    6400 6450
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B789D93
P 7900 5200
F 0 "R?" V 7980 5200 50  0000 C CNN
F 1 "47k" V 7900 5200 50  0000 C CNN
F 2 "" V 7830 5200 50  0001 C CNN
F 3 "" H 7900 5200 50  0001 C CNN
	1    7900 5200
	-1   0    0    1   
$EndComp
Text HLabel 800  700  0    60   Input ~ 0
VBAT
Text HLabel 8050 2050 2    60   Output ~ 0
~SWITCH
$Comp
L GND #PWR?
U 1 1 5B7E29C3
P 3150 3250
F 0 "#PWR?" H 3150 3000 50  0001 C CNN
F 1 "GND" H 3150 3100 50  0001 C CNN
F 2 "" H 3150 3250 50  0001 C CNN
F 3 "" H 3150 3250 50  0001 C CNN
	1    3150 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7E2AC7
P 5000 3250
F 0 "#PWR?" H 5000 3000 50  0001 C CNN
F 1 "GND" H 5000 3100 50  0001 C CNN
F 2 "" H 5000 3250 50  0001 C CNN
F 3 "" H 5000 3250 50  0001 C CNN
	1    5000 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7E2CC4
P 3900 3250
F 0 "#PWR?" H 3900 3000 50  0001 C CNN
F 1 "GND" H 3900 3100 50  0001 C CNN
F 2 "" H 3900 3250 50  0001 C CNN
F 3 "" H 3900 3250 50  0001 C CNN
	1    3900 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7E2DC8
P 4700 3250
F 0 "#PWR?" H 4700 3000 50  0001 C CNN
F 1 "GND" H 4700 3100 50  0001 C CNN
F 2 "" H 4700 3250 50  0001 C CNN
F 3 "" H 4700 3250 50  0001 C CNN
	1    4700 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7E4573
P 5950 2650
F 0 "#PWR?" H 5950 2400 50  0001 C CNN
F 1 "GND" H 5950 2500 50  0001 C CNN
F 2 "" H 5950 2650 50  0001 C CNN
F 3 "" H 5950 2650 50  0001 C CNN
	1    5950 2650
	1    0    0    -1  
$EndComp
Text HLabel 8050 3600 2    60   Input ~ 0
USENSE
$Comp
L GND #PWR?
U 1 1 5B7E9E77
P 3800 7600
F 0 "#PWR?" H 3800 7350 50  0001 C CNN
F 1 "GND" H 3800 7450 50  0001 C CNN
F 2 "" H 3800 7600 50  0001 C CNN
F 3 "" H 3800 7600 50  0001 C CNN
	1    3800 7600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7EA4AE
P 4050 7450
F 0 "#PWR?" H 4050 7200 50  0001 C CNN
F 1 "GND" H 4050 7300 50  0001 C CNN
F 2 "" H 4050 7450 50  0001 C CNN
F 3 "" H 4050 7450 50  0001 C CNN
	1    4050 7450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B14260D
P 4900 5850
F 0 "R?" V 4980 5850 50  0000 C CNN
F 1 "47k" V 4900 5850 50  0000 C CNN
F 2 "" V 4830 5850 50  0001 C CNN
F 3 "" H 4900 5850 50  0001 C CNN
	1    4900 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B7EEB32
P 6900 6350
F 0 "#PWR?" H 6900 6100 50  0001 C CNN
F 1 "GND" H 6900 6200 50  0001 C CNN
F 2 "" H 6900 6350 50  0001 C CNN
F 3 "" H 6900 6350 50  0001 C CNN
	1    6900 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7550 3800 7600
Connection ~ 3800 7200
Wire Wire Line
	3800 7200 3800 7250
Connection ~ 3800 7000
Wire Wire Line
	3800 6650 3800 7000
Connection ~ 3500 4450
Wire Wire Line
	3500 4450 3500 4500
Connection ~ 3550 5750
Wire Wire Line
	3200 5750 3600 5750
Wire Wire Line
	3250 5750 3250 5800
Wire Wire Line
	3550 5750 3550 5800
Wire Wire Line
	2850 7550 2850 7500
Wire Wire Line
	2650 7500 2650 7600
Wire Wire Line
	2650 7150 2650 7200
Wire Wire Line
	2650 6800 2650 6850
Wire Wire Line
	3750 7200 3850 7200
Wire Wire Line
	3750 7000 3850 7000
Wire Wire Line
	3250 7550 3250 7600
Connection ~ 3250 7000
Wire Wire Line
	3200 7000 3450 7000
Wire Wire Line
	3250 7000 3250 7250
Wire Wire Line
	4450 7100 6100 7100
Wire Wire Line
	3450 4250 3550 4250
Wire Wire Line
	4200 5650 4350 5650
Wire Wire Line
	4250 5650 4250 6350
Connection ~ 3250 5750
Connection ~ 4250 6150
Wire Wire Line
	4250 6350 4050 6350
Wire Wire Line
	4250 6150 4050 6150
Connection ~ 3550 6150
Wire Wire Line
	3550 6350 3750 6350
Wire Wire Line
	3550 6100 3550 6350
Wire Wire Line
	3750 6150 3550 6150
Connection ~ 2800 4350
Connection ~ 2150 2250
Wire Wire Line
	2150 3050 2150 3250
Wire Wire Line
	1650 2350 1650 2400
Wire Wire Line
	1650 1750 1650 2050
Wire Wire Line
	2150 1400 2150 1450
Wire Wire Line
	1650 1400 1650 1450
Wire Wire Line
	1850 2000 1650 2000
Wire Wire Line
	2150 1750 2150 1800
Wire Wire Line
	2150 2200 2150 2300
Wire Wire Line
	2150 2600 2150 2650
Connection ~ 1800 2850
Wire Wire Line
	1800 2850 1800 2900
Wire Wire Line
	1750 2850 1850 2850
Wire Wire Line
	1800 3200 1800 3250
Connection ~ 2800 4850
Wire Wire Line
	2800 4350 2850 4350
Wire Wire Line
	3450 4450 3550 4450
Wire Wire Line
	2800 5050 3000 5050
Connection ~ 3500 4850
Wire Wire Line
	3500 5050 3300 5050
Wire Wire Line
	3500 4800 3500 5050
Wire Wire Line
	3300 4850 3500 4850
Wire Wire Line
	5950 2600 5950 2600
Connection ~ 3900 1400
Wire Wire Line
	3900 1400 3900 1600
Connection ~ 3900 2000
Wire Wire Line
	4200 2400 4300 2400
Wire Wire Line
	3900 1900 3900 2200
Wire Wire Line
	3900 2000 3150 2000
Wire Wire Line
	3150 2000 3150 2200
Connection ~ 3900 2850
Wire Wire Line
	3900 2600 3900 2900
Wire Wire Line
	3900 3200 3900 3250
Connection ~ 4250 1400
Wire Wire Line
	3500 1400 3500 2050
Connection ~ 3150 2650
Wire Wire Line
	3150 2650 4400 2650
Wire Wire Line
	3150 2600 3150 2900
Wire Wire Line
	4250 3200 4250 3250
Wire Wire Line
	2600 1100 2600 1050
Wire Wire Line
	1500 1100 2800 1100
Wire Wire Line
	2400 1050 2400 1100
Wire Wire Line
	4700 1400 4700 1600
Wire Wire Line
	4700 1900 4700 1950
Connection ~ 4700 2400
Wire Wire Line
	4250 1400 4250 1600
Connection ~ 4250 2150
Wire Wire Line
	4250 2150 4400 2150
Wire Wire Line
	4250 1900 4250 2900
Wire Wire Line
	4700 2850 4700 2900
Wire Wire Line
	4700 2350 4700 2450
Wire Wire Line
	4600 2400 5350 2400
Wire Wire Line
	5650 2400 5750 2400
Wire Wire Line
	5650 2200 5750 2200
Connection ~ 6800 2300
Wire Wire Line
	6800 2300 6800 2250
Wire Wire Line
	6350 2300 6850 2300
Wire Wire Line
	7150 2850 7150 2900
Wire Wire Line
	7150 2500 7150 2550
Wire Wire Line
	5800 4150 5700 4150
Wire Wire Line
	5750 4350 5700 4350
Wire Wire Line
	5050 4250 5100 4250
Wire Wire Line
	6000 5650 6000 5700
Connection ~ 4250 5650
Wire Wire Line
	3400 7200 3450 7200
Wire Wire Line
	2650 6450 2650 6500
Wire Wire Line
	4500 6750 4500 7100
Connection ~ 4500 7100
Connection ~ 4600 7100
Wire Wire Line
	2800 7000 2900 7000
Wire Wire Line
	2850 5550 2850 7200
Connection ~ 2850 7000
Wire Wire Line
	4900 5100 4900 5200
Wire Wire Line
	5600 5100 5600 5200
Wire Wire Line
	5200 5400 5300 5400
Wire Wire Line
	4900 5600 4900 5700
Wire Wire Line
	4650 5650 4900 5650
Connection ~ 4900 5650
Wire Wire Line
	5750 4350 5750 3800
Wire Wire Line
	5750 3800 5600 3800
Wire Wire Line
	5250 5400 5250 5150
Wire Wire Line
	5250 5150 5850 5150
Connection ~ 5250 5400
Wire Wire Line
	6000 5300 6000 5350
Wire Wire Line
	6000 4750 6000 5000
Wire Wire Line
	5950 1400 5950 2000
Wire Wire Line
	6800 1400 6800 1950
Wire Wire Line
	1650 1400 8050 1400
Wire Wire Line
	3150 3200 3150 3250
Wire Wire Line
	6000 6050 6000 6450
Connection ~ 6000 6050
Wire Wire Line
	3850 4250 4750 4250
Wire Wire Line
	3500 2850 3550 2850
Wire Wire Line
	3500 2350 3500 2850
Wire Wire Line
	3500 2400 3450 2400
Wire Wire Line
	3850 2850 3900 2850
Connection ~ 3500 2400
Wire Wire Line
	2800 1100 2800 1050
Connection ~ 2600 1100
Wire Wire Line
	2800 1550 2800 4000
Wire Wire Line
	6650 5950 6950 5950
Wire Wire Line
	2150 2250 2800 2250
Wire Wire Line
	2350 700  3050 700 
Wire Wire Line
	2400 700  2400 750 
Wire Wire Line
	2600 700  2600 750 
Connection ~ 2400 700 
Wire Wire Line
	2800 700  2800 750 
Connection ~ 2600 700 
Wire Wire Line
	2050 1000 2050 1150
Connection ~ 2400 1100
Wire Wire Line
	1400 700  1750 700 
Wire Wire Line
	1700 700  1700 750 
Wire Wire Line
	1700 1100 1700 1050
Connection ~ 2050 1100
Connection ~ 2800 700 
Connection ~ 1700 700 
Wire Wire Line
	6400 2300 6400 2200
Connection ~ 6400 2300
Wire Wire Line
	6400 1750 6400 1900
Wire Wire Line
	5350 2200 5300 2200
Wire Wire Line
	5300 2200 5300 1550
Wire Wire Line
	6350 1750 6400 1750
Wire Wire Line
	6050 1750 5700 1750
Wire Wire Line
	5700 1750 5700 2200
Connection ~ 5700 2200
Connection ~ 1650 2000
Wire Wire Line
	1450 2850 1400 2850
Wire Wire Line
	1050 2850 1100 2850
Wire Wire Line
	6900 6300 6900 6350
Wire Wire Line
	6900 5950 6900 6000
Connection ~ 6900 5950
Wire Wire Line
	6550 6450 6700 6450
Wire Wire Line
	6000 6450 6250 6450
Wire Wire Line
	7900 5000 7900 5050
Connection ~ 7900 5000
Wire Wire Line
	7900 5350 7900 5400
Wire Wire Line
	2950 700  2950 1400
Connection ~ 2150 1400
Connection ~ 1050 700 
Wire Wire Line
	1250 2700 1250 2650
Wire Wire Line
	1250 2650 1050 2650
Connection ~ 1050 2650
Wire Wire Line
	1050 700  1050 5750
Connection ~ 3500 1400
Connection ~ 2950 1400
Wire Wire Line
	5000 2400 5000 2750
Connection ~ 5000 2400
Wire Wire Line
	5000 3250 5000 3050
Wire Wire Line
	5300 1550 2800 1550
Connection ~ 4700 1400
Connection ~ 5950 1400
Wire Wire Line
	7150 2100 7150 2050
Wire Wire Line
	7150 2050 8050 2050
Wire Wire Line
	4700 3200 4700 3250
Wire Wire Line
	5950 2600 5950 2650
Connection ~ 2800 2250
Wire Wire Line
	4050 7400 4050 7450
Wire Wire Line
	4700 5650 4700 4700
Wire Wire Line
	4900 4800 4900 4750
Wire Wire Line
	4900 4750 6550 4750
Wire Wire Line
	5600 4750 5600 4800
Wire Wire Line
	5600 5600 5600 5700
Wire Wire Line
	5400 5700 5400 5650
Wire Wire Line
	5400 5650 5800 5650
Connection ~ 5600 5650
Wire Wire Line
	4900 6000 4900 6050
Wire Wire Line
	4900 6050 6050 6050
Wire Wire Line
	5600 6050 5600 6000
Connection ~ 5600 4750
Connection ~ 5600 6050
Wire Wire Line
	6700 6450 6700 6150
Wire Wire Line
	6700 6150 6650 6150
Wire Wire Line
	5300 3800 4700 3800
Wire Wire Line
	4700 3800 4700 4400
Connection ~ 4700 4250
Wire Wire Line
	2800 4300 2800 5050
Wire Wire Line
	2800 4850 3000 4850
Wire Wire Line
	3850 4450 4250 4450
Wire Wire Line
	4250 4450 4250 3600
Wire Wire Line
	4250 3600 8050 3600
Connection ~ 4700 5650
Wire Wire Line
	5800 5650 5800 4700
Wire Wire Line
	5800 4150 5800 4400
Text Label 4700 5100 1    60   ~ 0
UREG
Text Label 5800 5100 1    60   ~ 0
UMIN
$Comp
L GND #PWR?
U 1 1 5B7FDB13
P 3250 6150
F 0 "#PWR?" H 3250 5900 50  0001 C CNN
F 1 "GND" H 3250 6000 50  0001 C CNN
F 2 "" H 3250 6150 50  0001 C CNN
F 3 "" H 3250 6150 50  0001 C CNN
	1    3250 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 6100 3250 6150
Wire Wire Line
	1050 5750 2900 5750
Connection ~ 1050 2850
Text HLabel 8050 5950 2    60   Input ~ 0
UREF
Wire Wire Line
	7250 5950 8050 5950
Wire Wire Line
	3400 5750 3400 7200
Connection ~ 3400 5750
Wire Wire Line
	4600 7100 4600 7000
Wire Wire Line
	3800 6650 4600 6650
Wire Wire Line
	4600 6650 4600 6700
Wire Wire Line
	4500 6750 4450 6750
Wire Wire Line
	3900 6750 4150 6750
Wire Wire Line
	4050 6750 4050 6800
Wire Wire Line
	5400 7100 5400 6000
$Comp
L GND #PWR?
U 1 1 5B803923
P 3250 7600
F 0 "#PWR?" H 3250 7350 50  0001 C CNN
F 1 "GND" H 3250 7450 50  0001 C CNN
F 2 "" H 3250 7600 50  0001 C CNN
F 3 "" H 3250 7600 50  0001 C CNN
	1    3250 7600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B8041DC
P 2650 7600
F 0 "#PWR?" H 2650 7350 50  0001 C CNN
F 1 "GND" H 2650 7450 50  0001 C CNN
F 2 "" H 2650 7600 50  0001 C CNN
F 3 "" H 2650 7600 50  0001 C CNN
	1    2650 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7550 2850 7550
Connection ~ 2650 7550
Wire Wire Line
	6450 4750 6450 5750
Connection ~ 6000 4750
Text Label 3050 700  0    60   ~ 0
+5V
Connection ~ 2950 700 
Text Label 6550 4750 0    60   ~ 0
+5V
Connection ~ 6450 4750
Text Label 3900 6750 0    60   ~ 0
+5V
Connection ~ 4050 6750
Text Label 2650 6450 0    60   ~ 0
+5V
Wire Wire Line
	6450 5000 7550 5000
Text HLabel 8050 5000 2    60   Output ~ 0
USET
Wire Wire Line
	7850 5000 8050 5000
Text Label 5000 2400 0    60   ~ 0
USAW
$Comp
L R R?
U 1 1 5B809E8D
P 1250 700
F 0 "R?" V 1330 700 50  0000 C CNN
F 1 "100" V 1250 700 50  0000 C CNN
F 2 "" V 1180 700 50  0001 C CNN
F 3 "" H 1250 700 50  0001 C CNN
	1    1250 700 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	800  700  1100 700 
$Comp
L D_TVS D?
U 1 1 5B80AF2D
P 1500 900
F 0 "D?" H 1500 1000 50  0000 C CNN
F 1 "D_TVS" H 1500 800 50  0000 C CNN
F 2 "" H 1500 900 50  0001 C CNN
F 3 "" H 1500 900 50  0001 C CNN
	1    1500 900 
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 700  1500 750 
Connection ~ 1500 700 
Wire Wire Line
	1500 1050 1500 1100
Connection ~ 1700 1100
Connection ~ 6450 5000
$Comp
L GND #PWR?
U 1 1 5B80D4A8
P 7900 5400
F 0 "#PWR?" H 7900 5150 50  0001 C CNN
F 1 "GND" H 7900 5250 50  0001 C CNN
F 2 "" H 7900 5400 50  0001 C CNN
F 3 "" H 7900 5400 50  0001 C CNN
	1    7900 5400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5B86F8C3
P 3050 5550
F 0 "R?" V 3130 5550 50  0000 C CNN
F 1 "10k" V 3050 5550 50  0000 C CNN
F 2 "" V 2980 5550 50  0001 C CNN
F 3 "" H 3050 5550 50  0001 C CNN
	1    3050 5550
	0    -1   1    0   
$EndComp
Wire Wire Line
	2850 5550 2900 5550
Wire Wire Line
	3200 5550 3600 5550
Connection ~ 6800 1400
Text HLabel 8050 1400 2    60   Output ~ 0
+5V
Text HLabel 6100 7100 2    60   Output ~ 0
CHARGED
Connection ~ 5400 7100
$EndSCHEMATC
