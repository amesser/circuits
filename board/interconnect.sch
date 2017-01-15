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
Sheet 3 6
Title "Altera MAX 10 FPGA Development Board"
Date "2017-01-14"
Rev "1"
Comp "Copyright (c) 2017 Andreas Messer"
Comment1 "MAX10-SAM3U Interconnect"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 10M08DCF256-RESCUE-data-logger U?
U 1 1 5864DFF8
P 2000 4700
AR Path="/5864DFF8" Ref="U?"  Part="1" 
AR Path="/5864D98F/5864DFF8" Ref="U?"  Part="1" 
F 0 "U?" H 2000 4700 60  0000 C CNN
F 1 "10M08DCF256" H 2000 3800 60  0000 C CNN
F 2 "" H 2000 3500 60  0001 C CNN
F 3 "" H 2000 3500 60  0001 C CNN
	1    2000 4700
	1    0    0    -1  
$EndComp
$Comp
L SAM3U1EB-CU U?
U 2 1 5864E288
P 4350 2400
F 0 "U?" H 4100 800 60  0000 C CNN
F 1 "SAM3U1EB-CU" H 4350 700 60  0000 C CNN
F 2 "" H 5200 1300 60  0001 C CNN
F 3 "" H 5200 1300 60  0001 C CNN
	2    4350 2400
	1    0    0    -1  
$EndComp
Text Label 5600 3400 0    60   ~ 0
SMC_D8
Text Label 5600 3500 0    60   ~ 0
SMC_D9
Text Label 5600 3600 0    60   ~ 0
SMC_D10
Text Label 5600 3700 0    60   ~ 0
SMC_D11
Text Label 5600 3800 0    60   ~ 0
SMC_D12
Text Label 5600 3900 0    60   ~ 0
SMC_D13
Text Label 5600 4000 0    60   ~ 0
SMC_D14
Text Label 5600 1500 0    60   ~ 0
SMC_D15
Text Label 5600 1800 0    60   ~ 0
SMC_D0
Text Label 5600 1900 0    60   ~ 0
SMC_D1
Text Label 5600 2000 0    60   ~ 0
SMC_D2
Text Label 5600 2100 0    60   ~ 0
SMC_D3
Text Label 5600 2200 0    60   ~ 0
SMC_D4
Text Label 5600 2300 0    60   ~ 0
SMC_D5
Text Label 5600 2400 0    60   ~ 0
SMC_D6
Text Label 5600 2500 0    60   ~ 0
SMC_D7
Text Label 5600 1700 0    60   ~ 0
SMC_A1
Text Label 5600 4100 0    60   ~ 0
SMC_A2
Text Label 5600 4200 0    60   ~ 0
SMC_A3
Text Label 5600 4300 0    60   ~ 0
SMC_A4
Text Label 5600 4400 0    60   ~ 0
SMC_A5
Text Label 5600 4500 0    60   ~ 0
SMC_A6
Text Label 5600 4600 0    60   ~ 0
SMC_A7
$Comp
L SAM3U1EB-CU U?
U 3 1 5864E8FF
P 4350 5600
F 0 "U?" H 4100 4000 60  0000 C CNN
F 1 "SAM3U1EB-CU" H 4350 3900 60  0000 C CNN
F 2 "" H 5200 4500 60  0001 C CNN
F 3 "" H 5200 4500 60  0001 C CNN
	3    4350 5600
	1    0    0    -1  
$EndComp
Text Label 5600 4700 0    60   ~ 0
SMC_A8
Text Label 5600 4800 0    60   ~ 0
SMC_A9
Text Label 5600 4900 0    60   ~ 0
SMC_A10
Text Label 5600 5000 0    60   ~ 0
SMC_A11
Text Label 5600 5100 0    60   ~ 0
SMC_A12
Text Label 5600 5200 0    60   ~ 0
SMC_A13
Text Label 5600 6100 0    60   ~ 0
SMC_A14
Text Label 5600 6200 0    60   ~ 0
SMC_A15
Text Label 5600 2900 0    60   ~ 0
SMC_NCS0
Text Label 5600 1600 0    60   ~ 0
SMC_NBS0
Text Label 5600 2800 0    60   ~ 0
SMC_NRD
Text Label 5600 3200 0    60   ~ 0
SMC_NWE
Text Label 5600 5600 0    60   ~ 0
SMC_NBS1
Text Label 5600 5700 0    60   ~ 0
SMC_NCS2
Text Label 2600 4800 0    60   ~ 0
SMC_D8
Text Label 2600 4900 0    60   ~ 0
SMC_D9
Text Label 2600 5000 0    60   ~ 0
SMC_D10
Text Label 2600 5100 0    60   ~ 0
SMC_D11
Text Label 2600 5200 0    60   ~ 0
SMC_D12
Text Label 2600 5300 0    60   ~ 0
SMC_D13
Text Label 2600 5400 0    60   ~ 0
SMC_D14
Text Label 2600 5500 0    60   ~ 0
SMC_D15
Text Label 2600 4000 0    60   ~ 0
SMC_D0
Text Label 2600 4100 0    60   ~ 0
SMC_D1
Text Label 2600 4200 0    60   ~ 0
SMC_D2
Text Label 2600 4300 0    60   ~ 0
SMC_D3
Text Label 2600 4400 0    60   ~ 0
SMC_D4
Text Label 2600 4500 0    60   ~ 0
SMC_D5
Text Label 2600 4600 0    60   ~ 0
SMC_D6
Text Label 2600 4700 0    60   ~ 0
SMC_D7
Text Label 2600 1600 0    60   ~ 0
SMC_A1
Text Label 2600 1800 0    60   ~ 0
SMC_A2
Text Label 2600 1900 0    60   ~ 0
SMC_A3
Text Label 2600 2000 0    60   ~ 0
SMC_A4
Text Label 2600 2100 0    60   ~ 0
SMC_A5
Text Label 2600 2300 0    60   ~ 0
SMC_A6
Text Label 2600 2400 0    60   ~ 0
SMC_A7
Text Label 2600 2500 0    60   ~ 0
SMC_A8
Text Label 2600 2600 0    60   ~ 0
SMC_A9
Text Label 2600 2700 0    60   ~ 0
SMC_A10
Text Label 2600 2800 0    60   ~ 0
SMC_A11
Text Label 2600 3000 0    60   ~ 0
SMC_A12
Text Label 2600 3100 0    60   ~ 0
SMC_A13
Text Label 2600 3200 0    60   ~ 0
SMC_A14
Text Label 2600 3500 0    60   ~ 0
SMC_A15
Text Label 2600 1100 0    60   ~ 0
SMC_NCS0
Text Label 2600 1500 0    60   ~ 0
SMC_NBS0
Text Label 2600 1200 0    60   ~ 0
SMC_NRD
Text Label 2600 1300 0    60   ~ 0
SMC_NWE
Text Label 2600 1400 0    60   ~ 0
SMC_NBS1
Text Label 2600 1000 0    60   ~ 0
SMC_NCS2
$Comp
L C C?
U 1 1 5864ECDB
P 1100 1100
F 0 "C?" H 1125 1200 50  0000 L CNN
F 1 "4.7µF" H 1125 1000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1138 950 50  0001 C CNN
F 3 "" H 1100 1100 50  0000 C CNN
	1    1100 1100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5864EF90
P 800 1100
F 0 "C?" H 825 1200 50  0000 L CNN
F 1 "4.7µF" H 825 1000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 838 950 50  0001 C CNN
F 3 "" H 800 1100 50  0000 C CNN
	1    800  1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5864F2BA
P 950 1350
F 0 "#PWR08" H 950 1100 50  0001 C CNN
F 1 "GND" H 950 1200 50  0001 C CNN
F 2 "" H 950 1350 50  0000 C CNN
F 3 "" H 950 1350 50  0000 C CNN
	1    950  1350
	1    0    0    -1  
$EndComp
Text Label 2600 2200 0    60   ~ 0
FPGA_DEV_OE
Text Label 5600 3100 0    60   ~ 0
FPGA_DEV_OE
Text Label 2600 3900 0    60   ~ 0
FPGA_CONFIG_SEL
Text Label 5600 6600 0    60   ~ 0
FPGA_CONFIG_SEL
Text Label 2600 3400 0    60   ~ 0
~FPGA_STATUS~
Text Label 5600 2700 0    60   ~ 0
~FPGA_STATUS~
Text Label 650  3900 0    60   ~ 0
~FPGA_CONFIG~
Text Label 5600 2600 0    60   ~ 0
~FPGA_CONFIG~
Text Label 2600 3300 0    60   ~ 0
FPGA_CONF_DONE
Text Label 5600 6700 0    60   ~ 0
FPGA_CONF_DONE
Text Label 2600 2900 0    60   ~ 0
FPGA_CRC_ERROR
Text Label 5600 6500 0    60   ~ 0
FPGA_CRC_ERROR
Text Label 2600 1700 0    60   ~ 0
~FPGA_CLR~
Text Label 5600 3000 0    60   ~ 0
~FPGA_CLR~
Text GLabel 1250 7200 0    60   Input ~ 0
VCCIO_MCU
Text Label 650  900  0    60   ~ 0
VCCIO_MCU
Text Label 1750 7200 2    60   ~ 0
VCCIO_MCU
Text Label 1450 5900 0    60   ~ 0
VCCIO_MCU
Text Label 2600 6200 0    40   ~ 0
FPGA_CRC_ERROR
Text Label 2600 6100 0    40   ~ 0
FPGA_CONF_DONE
Text Label 2600 6000 0    40   ~ 0
~FPGA_STATUS~
Text Label 2600 5900 0    40   ~ 0
~FPGA_CLR~
Text Label 2600 6300 0    40   ~ 0
~FPGA_CONFIG~
Text Label 2600 6500 0    40   ~ 0
FPGA_DEV_OE
Text Label 2600 6600 0    40   ~ 0
FPGA_CONFIG_SEL
$Comp
L GND #PWR010
U 1 1 5865834A
P 1950 6650
F 0 "#PWR010" H 1950 6400 50  0001 C CNN
F 1 "GND" H 1950 6500 50  0001 C CNN
F 2 "" H 1950 6650 50  0000 C CNN
F 3 "" H 1950 6650 50  0000 C CNN
	1    1950 6650
	1    0    0    -1  
$EndComp
Entry Wire Line
	3400 1300 3500 1400
Entry Wire Line
	3400 1400 3500 1500
Entry Wire Line
	3400 1500 3500 1600
Entry Wire Line
	3400 1600 3500 1700
Entry Wire Line
	3400 1700 3500 1800
Entry Wire Line
	3400 1800 3500 1900
Entry Wire Line
	3400 1900 3500 2000
Entry Wire Line
	3400 2000 3500 2100
Entry Wire Line
	3400 2100 3500 2200
Entry Wire Line
	3400 2200 3500 2300
Entry Wire Line
	3400 2300 3500 2400
Entry Wire Line
	3400 2400 3500 2500
Entry Wire Line
	3400 2500 3500 2600
Entry Wire Line
	3400 2600 3500 2700
Entry Wire Line
	3400 2700 3500 2800
Entry Wire Line
	3400 2800 3500 2900
Entry Wire Line
	3400 2900 3500 3000
Entry Wire Line
	3400 3000 3500 3100
Entry Wire Line
	3400 3100 3500 3200
Entry Wire Line
	3400 3200 3500 3300
Entry Wire Line
	3400 3300 3500 3400
Entry Wire Line
	3400 3400 3500 3500
Entry Wire Line
	3400 3500 3500 3600
Entry Wire Line
	3400 3600 3500 3700
Entry Wire Line
	3400 3700 3500 3800
Entry Wire Line
	3400 3800 3500 3900
Entry Wire Line
	3400 3900 3500 4000
Entry Wire Line
	3400 4000 3500 4100
Entry Wire Line
	3400 4100 3500 4200
Entry Wire Line
	3400 4200 3500 4300
Entry Wire Line
	3400 4300 3500 4400
Entry Wire Line
	3400 4400 3500 4500
Entry Wire Line
	3400 4500 3500 4600
Entry Wire Line
	3400 4600 3500 4700
Entry Wire Line
	3400 4700 3500 4800
Entry Wire Line
	3400 4800 3500 4900
Entry Wire Line
	3400 4900 3500 5000
Entry Wire Line
	3400 5000 3500 5100
Entry Wire Line
	3400 5100 3500 5200
Entry Wire Line
	3400 5200 3500 5300
Entry Wire Line
	3400 5300 3500 5400
Entry Wire Line
	3400 5400 3500 5500
Entry Wire Line
	3400 5500 3500 5600
Entry Wire Line
	3400 5900 3500 6000
Entry Wire Line
	3400 6000 3500 6100
Entry Wire Line
	3400 6100 3500 6200
Entry Wire Line
	3400 6200 3500 6300
Entry Wire Line
	3400 6300 3500 6400
Entry Wire Line
	3400 6500 3500 6600
Entry Wire Line
	3400 6600 3500 6700
Entry Wire Line
	3400 5700 3500 5800
Entry Wire Line
	6400 1500 6500 1600
Entry Wire Line
	6400 1600 6500 1700
Entry Wire Line
	6400 1700 6500 1800
Entry Wire Line
	6400 1800 6500 1900
Entry Wire Line
	6400 1900 6500 2000
Entry Wire Line
	6400 2000 6500 2100
Entry Wire Line
	6400 2100 6500 2200
Entry Wire Line
	6400 2200 6500 2300
Entry Wire Line
	6400 2300 6500 2400
Entry Wire Line
	6400 2400 6500 2500
Entry Wire Line
	6400 2500 6500 2600
Entry Wire Line
	6400 2600 6500 2700
Entry Wire Line
	6400 2700 6500 2800
Entry Wire Line
	6400 2800 6500 2900
Entry Wire Line
	6400 2900 6500 3000
Entry Wire Line
	6400 3000 6500 3100
Entry Wire Line
	6400 3100 6500 3200
Entry Wire Line
	6400 3200 6500 3300
Entry Wire Line
	6400 3400 6500 3500
Entry Wire Line
	6400 3500 6500 3600
Entry Wire Line
	6400 3600 6500 3700
Entry Wire Line
	6400 3700 6500 3800
Entry Wire Line
	6400 3800 6500 3900
Entry Wire Line
	6400 3900 6500 4000
Entry Wire Line
	6400 4000 6500 4100
Entry Wire Line
	6400 4100 6500 4200
Entry Wire Line
	6400 4200 6500 4300
Entry Wire Line
	6400 4300 6500 4400
Entry Wire Line
	6400 4400 6500 4500
Entry Wire Line
	6400 4500 6500 4600
Entry Wire Line
	6400 4600 6500 4700
Entry Wire Line
	6400 4700 6500 4800
Entry Wire Line
	6400 4800 6500 4900
Entry Wire Line
	6400 4900 6500 5000
Entry Wire Line
	6400 5000 6500 5100
Entry Wire Line
	6400 5600 6500 5700
Entry Wire Line
	6400 5700 6500 5800
Entry Wire Line
	6400 6600 6500 6700
Entry Wire Line
	6400 5100 6500 5200
Entry Wire Line
	6400 6100 6500 6200
Entry Wire Line
	6400 6200 6500 6300
Text GLabel 6000 3300 2    50   Output ~ 0
MCU_PCK1
Text GLabel 5700 6900 2    50   Output ~ 0
MCU_FPGA_TCK
Text GLabel 5700 6800 2    50   Output ~ 0
MCU_FPGA_TMS
Text GLabel 5700 7000 2    50   Output ~ 0
MCU_FPGA_TDI
Text GLabel 5700 7100 2    50   Input ~ 0
MCU_FPGA_TDO
Entry Wire Line
	3400 1000 3500 1100
Entry Wire Line
	3400 1100 3500 1200
Entry Wire Line
	3400 1200 3500 1300
Text Label 2600 3600 0    60   ~ 0
SMC_A16
Text Label 2600 3700 0    60   ~ 0
SMC_A17
Text Label 5600 6300 0    60   ~ 0
SMC_A16
Text Label 5600 6400 0    60   ~ 0
SMC_A17
Entry Wire Line
	6400 6300 6500 6400
Entry Wire Line
	6400 6400 6500 6500
$Comp
L Led_Small D?
U 1 1 586824FF
P 5000 900
F 0 "D?" H 5050 950 50  0000 L CNN
F 1 "Led_Small" H 4825 800 50  0001 L CNN
F 2 "" V 5000 900 50  0000 C CNN
F 3 "" V 5000 900 50  0000 C CNN
	1    5000 900 
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D?
U 1 1 58682567
P 5000 1000
F 0 "D?" H 5050 1050 50  0000 L CNN
F 1 "Led_Small" H 4825 900 50  0001 L CNN
F 2 "" V 5000 1000 50  0000 C CNN
F 3 "" V 5000 1000 50  0000 C CNN
	1    5000 1000
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D?
U 1 1 5868259B
P 5000 1100
F 0 "D?" H 5050 1150 50  0000 L CNN
F 1 "Led_Small" H 4825 1000 50  0001 L CNN
F 2 "" V 5000 1100 50  0000 C CNN
F 3 "" V 5000 1100 50  0000 C CNN
	1    5000 1100
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D?
U 1 1 586825D2
P 5000 1200
F 0 "D?" H 5050 1250 50  0000 L CNN
F 1 "Led_Small" H 4825 1100 50  0001 L CNN
F 2 "" V 5000 1200 50  0000 C CNN
F 3 "" V 5000 1200 50  0000 C CNN
	1    5000 1200
	1    0    0    -1  
$EndComp
Text Label 5300 700  0    60   ~ 0
VCCIO_MCU
Text Label 4850 5300 0    60   ~ 0
MCU_TXD3
Text Label 4850 5400 0    60   ~ 0
MCU_RXD3
Text Label 4850 6000 0    60   ~ 0
MCU_SCK3
Text GLabel 4900 6900 2    50   Input ~ 0
MCU_AD4
Text GLabel 4900 7000 2    50   Input ~ 0
MCU_AD5
Text GLabel 4900 7100 2    50   Input ~ 0
MCU_AD6
Text GLabel 4900 7200 2    50   Input ~ 0
MCU_AD7
Text Label 5600 5900 0    60   ~ 0
SMC_NWAIT
Entry Wire Line
	6400 5200 6500 5300
Text Label 2600 3800 0    60   ~ 0
SMC_NWAIT
Entry Wire Line
	6400 6500 6500 6600
Wire Wire Line
	1350 1100 1400 1100
Wire Wire Line
	1350 900  1350 1100
Wire Wire Line
	1350 1000 1400 1000
Wire Wire Line
	650  900  1400 900 
Connection ~ 1350 1000
Wire Wire Line
	1100 950  1100 900 
Connection ~ 1350 900 
Connection ~ 1100 900 
Wire Wire Line
	1100 1300 1100 1250
Wire Wire Line
	800  1300 1100 1300
Wire Wire Line
	700  4000 1400 4000
Wire Wire Line
	1350 4100 1400 4100
Wire Wire Line
	1350 4100 1350 4000
Connection ~ 1350 4000
Wire Wire Line
	2600 1000 3400 1000
Wire Wire Line
	2600 1100 3400 1100
Wire Wire Line
	2600 1200 3400 1200
Wire Wire Line
	2600 1600 3400 1600
Wire Wire Line
	2600 1400 3400 1400
Wire Wire Line
	2600 1900 3400 1900
Wire Wire Line
	2600 2000 3400 2000
Wire Wire Line
	2600 2100 3400 2100
Wire Wire Line
	2600 2300 3400 2300
Wire Wire Line
	2600 2400 3400 2400
Wire Wire Line
	2600 2500 3400 2500
Wire Wire Line
	2600 2600 3400 2600
Wire Wire Line
	2600 2700 3400 2700
Wire Wire Line
	2600 2800 3400 2800
Wire Wire Line
	2600 3000 3400 3000
Wire Wire Line
	2600 3100 3400 3100
Wire Wire Line
	2600 3200 3400 3200
Wire Wire Line
	2600 3500 3400 3500
Wire Wire Line
	2600 3600 3400 3600
Wire Wire Line
	2600 3700 3400 3700
Wire Wire Line
	2600 3800 3400 3800
Wire Wire Line
	2600 4000 3400 4000
Wire Wire Line
	2600 4100 3400 4100
Wire Wire Line
	2600 4200 3400 4200
Wire Wire Line
	2600 4300 3400 4300
Wire Wire Line
	2600 4400 3400 4400
Wire Wire Line
	2600 4500 3400 4500
Wire Wire Line
	2600 4600 3400 4600
Wire Wire Line
	2600 4700 3400 4700
Wire Wire Line
	2600 4800 3400 4800
Wire Wire Line
	2600 4900 3400 4900
Wire Wire Line
	2600 5000 3400 5000
Wire Wire Line
	2600 5100 3400 5100
Wire Wire Line
	2600 5200 3400 5200
Wire Wire Line
	2600 5300 3400 5300
Wire Wire Line
	2600 5400 3400 5400
Wire Wire Line
	2600 5500 3400 5500
Wire Wire Line
	2600 2200 3400 2200
Wire Wire Line
	2600 3900 3400 3900
Wire Wire Line
	2600 3400 3400 3400
Wire Wire Line
	550  3900 1400 3900
Wire Wire Line
	3400 3300 2600 3300
Wire Wire Line
	3400 2900 2600 2900
Wire Wire Line
	2600 1700 3400 1700
Wire Wire Line
	1750 7200 1250 7200
Wire Wire Line
	1450 5900 2000 5900
Wire Wire Line
	1950 6000 2000 6000
Connection ~ 1950 5900
Wire Wire Line
	1950 6100 2000 6100
Connection ~ 1950 6000
Wire Wire Line
	1950 6200 2000 6200
Connection ~ 1950 6100
Connection ~ 1950 6200
Wire Wire Line
	2000 6600 1950 6600
Wire Wire Line
	1950 6500 1950 6650
Wire Wire Line
	2000 6500 1950 6500
Connection ~ 1950 6600
Wire Wire Line
	2300 5900 3400 5900
Wire Wire Line
	2300 6000 3400 6000
Wire Wire Line
	2300 6100 3400 6100
Wire Wire Line
	2300 6200 3400 6200
Wire Wire Line
	2300 6500 3400 6500
Wire Wire Line
	2300 6600 3400 6600
Wire Wire Line
	550  3900 550  5700
Wire Wire Line
	550  5700 3400 5700
Wire Bus Line
	3500 1100 3500 7500
Wire Wire Line
	4850 900  4900 900 
Wire Wire Line
	4900 1000 4850 1000
Wire Wire Line
	4850 1100 4900 1100
Wire Wire Line
	4900 1200 4850 1200
Wire Wire Line
	2600 1300 3400 1300
Wire Wire Line
	2600 1500 3400 1500
Wire Wire Line
	2600 1800 3400 1800
Wire Wire Line
	5100 900  5250 900 
Wire Wire Line
	5100 1000 5250 1000
Wire Wire Line
	5100 1100 5250 1100
Wire Wire Line
	5100 1200 5250 1200
Wire Wire Line
	5300 700  5850 700 
Wire Wire Line
	5850 700  5850 1200
Wire Wire Line
	5850 900  5550 900 
Wire Wire Line
	5850 1000 5550 1000
Connection ~ 5850 900 
Wire Wire Line
	5850 1100 5550 1100
Connection ~ 5850 1000
Wire Wire Line
	5850 1200 5550 1200
Connection ~ 5850 1100
Wire Wire Line
	4900 6900 4850 6900
Wire Wire Line
	4900 7000 4850 7000
Wire Wire Line
	4850 7100 4900 7100
Wire Wire Line
	4900 7200 4850 7200
Wire Wire Line
	4850 5600 6400 5600
Wire Wire Line
	4850 5100 6400 5100
Wire Wire Line
	4850 5200 6400 5200
Wire Wire Line
	4850 1500 6400 1500
Wire Wire Line
	4850 1600 6400 1600
Wire Wire Line
	4850 1700 6400 1700
Wire Wire Line
	4850 1800 6400 1800
Wire Wire Line
	4850 1900 6400 1900
Wire Wire Line
	4850 2000 6400 2000
Wire Wire Line
	4850 2100 6400 2100
Wire Wire Line
	4850 2200 6400 2200
Wire Wire Line
	4850 2300 6400 2300
Wire Wire Line
	4850 2400 6400 2400
Wire Wire Line
	4850 2500 6400 2500
Wire Wire Line
	4850 2600 6400 2600
Wire Wire Line
	4850 2700 6400 2700
Wire Wire Line
	4850 2800 6400 2800
Wire Wire Line
	4850 2900 6400 2900
Wire Wire Line
	4850 3000 6400 3000
Wire Wire Line
	4850 3100 6400 3100
Wire Wire Line
	4850 3200 6400 3200
Wire Wire Line
	4850 3300 6000 3300
Wire Wire Line
	4850 3400 6400 3400
Wire Wire Line
	4850 3500 6400 3500
Wire Wire Line
	4850 3600 6400 3600
Wire Wire Line
	4850 3700 6400 3700
Wire Wire Line
	4850 3800 6400 3800
Wire Wire Line
	4850 3900 6400 3900
Wire Wire Line
	4850 4000 6400 4000
Wire Wire Line
	4850 4100 6400 4100
Wire Wire Line
	4850 4200 6400 4200
Wire Wire Line
	4850 4300 6400 4300
Wire Wire Line
	4850 4400 6400 4400
Wire Wire Line
	4850 4500 6400 4500
Wire Wire Line
	4850 4600 6400 4600
Wire Wire Line
	4850 4700 6400 4700
Wire Wire Line
	4850 4800 6400 4800
Wire Wire Line
	4850 4900 6400 4900
Wire Wire Line
	4850 5000 6400 5000
Wire Wire Line
	4850 5700 6400 5700
Wire Wire Line
	4850 5900 6400 5900
Wire Wire Line
	4850 6200 6400 6200
Wire Wire Line
	4850 6300 6400 6300
Wire Wire Line
	4850 6400 6400 6400
Wire Wire Line
	5700 6900 5450 6900
Wire Wire Line
	5450 6900 5450 6000
Wire Wire Line
	5450 6000 4850 6000
Wire Wire Line
	4850 5300 5400 5300
Wire Wire Line
	5400 5300 5400 7000
Wire Wire Line
	5400 7000 5700 7000
Wire Wire Line
	5700 7100 5350 7100
Wire Wire Line
	5350 7100 5350 5400
Wire Wire Line
	5350 5400 4850 5400
Wire Wire Line
	4850 6100 6400 6100
Wire Wire Line
	4850 6500 6400 6500
Wire Wire Line
	4850 6800 5700 6800
Wire Wire Line
	4850 6600 6400 6600
Wire Wire Line
	4850 6700 6400 6700
Entry Wire Line
	6400 6700 6500 6800
Entry Wire Line
	6400 5900 6500 6000
Wire Bus Line
	3500 7500 6500 7500
Wire Bus Line
	6500 7500 6500 1600
$Comp
L R_ARRAY_4 R?
U 1 1 58740B74
P 5400 900
F 0 "R?" V 5350 1100 60  0000 C CNN
F 1 "330" V 5400 900 60  0000 C CNN
F 2 "" H 5400 900 60  0001 C CNN
F 3 "" H 5400 900 60  0001 C CNN
	1    5400 900 
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 2 1 58740CDF
P 5400 1000
F 0 "R?" V 5350 1200 60  0000 C CNN
F 1 "330" V 5400 1000 60  0000 C CNN
F 2 "" H 5400 1000 60  0001 C CNN
F 3 "" H 5400 1000 60  0001 C CNN
	2    5400 1000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 3 1 58740D1F
P 5400 1100
F 0 "R?" V 5350 1300 60  0000 C CNN
F 1 "330" V 5400 1100 60  0000 C CNN
F 2 "" H 5400 1100 60  0001 C CNN
F 3 "" H 5400 1100 60  0001 C CNN
	3    5400 1100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 4 1 58740D66
P 5400 1200
F 0 "R?" V 5350 1400 60  0000 C CNN
F 1 "330" V 5400 1200 60  0000 C CNN
F 2 "" H 5400 1200 60  0001 C CNN
F 3 "" H 5400 1200 60  0001 C CNN
	4    5400 1200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 58740F2F
P 2150 5900
F 0 "R?" V 2100 6100 60  0000 C CNN
F 1 "10k" V 2150 5900 60  0000 C CNN
F 2 "" H 2150 5900 60  0001 C CNN
F 3 "" H 2150 5900 60  0001 C CNN
	1    2150 5900
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 2 1 5874109F
P 2150 6000
F 0 "R?" V 2100 6200 60  0000 C CNN
F 1 "10k" V 2150 6000 60  0000 C CNN
F 2 "" H 2150 6000 60  0001 C CNN
F 3 "" H 2150 6000 60  0001 C CNN
	2    2150 6000
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 3 1 587410E5
P 2150 6100
F 0 "R?" V 2100 6300 60  0000 C CNN
F 1 "10k" V 2150 6100 60  0000 C CNN
F 2 "" H 2150 6100 60  0001 C CNN
F 3 "" H 2150 6100 60  0001 C CNN
	3    2150 6100
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 4 1 5874112E
P 2150 6200
F 0 "R?" V 2100 6400 60  0000 C CNN
F 1 "10k" V 2150 6200 60  0000 C CNN
F 2 "" H 2150 6200 60  0001 C CNN
F 3 "" H 2150 6200 60  0001 C CNN
	4    2150 6200
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 1 1 5874117A
P 2150 6300
F 0 "R?" V 2100 6500 60  0000 C CNN
F 1 "10k" V 2150 6300 60  0000 C CNN
F 2 "" H 2150 6300 60  0001 C CNN
F 3 "" H 2150 6300 60  0001 C CNN
	1    2150 6300
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 2 1 587411C9
P 2150 6500
F 0 "R?" V 2100 6700 60  0000 C CNN
F 1 "10k" V 2150 6500 60  0000 C CNN
F 2 "" H 2150 6500 60  0001 C CNN
F 3 "" H 2150 6500 60  0001 C CNN
	2    2150 6500
	0    1    1    0   
$EndComp
$Comp
L R_ARRAY_4 R?
U 3 1 58741227
P 2150 6600
F 0 "R?" V 2100 6800 60  0000 C CNN
F 1 "10k" V 2150 6600 60  0000 C CNN
F 2 "" H 2150 6600 60  0001 C CNN
F 3 "" H 2150 6600 60  0001 C CNN
	3    2150 6600
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 5900 1950 6300
Wire Wire Line
	1950 6300 2000 6300
Wire Wire Line
	2300 6300 3400 6300
Wire Wire Line
	800  1250 800  1300
Wire Wire Line
	950  1350 950  1300
Connection ~ 950  1300
Wire Wire Line
	800  950  800  900 
Connection ~ 800  900 
$Comp
L C C?
U 1 1 5876AC00
P 1150 4200
F 0 "C?" H 1175 4300 50  0000 L CNN
F 1 "4.7µF" H 1175 4100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1188 4050 50  0001 C CNN
F 3 "" H 1150 4200 50  0000 C CNN
	1    1150 4200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5876AC06
P 850 4200
F 0 "C?" H 875 4300 50  0000 L CNN
F 1 "4.7µF" H 875 4100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 888 4050 50  0001 C CNN
F 3 "" H 850 4200 50  0000 C CNN
	1    850  4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5876AC0C
P 1000 4450
F 0 "#PWR?" H 1000 4200 50  0001 C CNN
F 1 "GND" H 1000 4300 50  0001 C CNN
F 2 "" H 1000 4450 50  0000 C CNN
F 3 "" H 1000 4450 50  0000 C CNN
	1    1000 4450
	1    0    0    -1  
$EndComp
Text Label 700  4000 0    60   ~ 0
VCCIO_MCU
Wire Wire Line
	1150 4050 1150 4000
Connection ~ 1150 4000
Wire Wire Line
	1150 4400 1150 4350
Wire Wire Line
	850  4400 1150 4400
Wire Wire Line
	850  4350 850  4400
Wire Wire Line
	1000 4450 1000 4400
Connection ~ 1000 4400
Wire Wire Line
	850  4050 850  4000
Connection ~ 850  4000
$Comp
L 10M08DCF256 U?
U 9 1 587BEE4F
P 2000 2400
F 0 "U?" H 2000 2600 60  0000 C CNN
F 1 "10M08DCF256" H 2000 550 60  0000 C CNN
F 2 "" H 2000 1200 60  0001 C CNN
F 3 "" H 2000 1200 60  0001 C CNN
	9    2000 2400
	1    0    0    -1  
$EndComp
$EndSCHEMATC
