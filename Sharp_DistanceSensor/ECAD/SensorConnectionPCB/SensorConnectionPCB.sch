EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Connector:Conn_01x03_Female J1
U 1 1 5BBC59C8
P 1900 3500
F 0 "J1" H 1794 3175 50  0000 C CNN
F 1 "Conn_01x03_Female" H 1794 3266 50  0000 C CNN
F 2 "" H 1900 3500 50  0001 C CNN
F 3 "~" H 1900 3500 50  0001 C CNN
	1    1900 3500
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5BBC5A63
P 2250 4000
F 0 "#PWR03" H 2250 3750 50  0001 C CNN
F 1 "GND" H 2255 3827 50  0000 C CNN
F 2 "" H 2250 4000 50  0001 C CNN
F 3 "" H 2250 4000 50  0001 C CNN
	1    2250 4000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5BBC5ACE
P 2250 2900
F 0 "#PWR01" H 2250 2750 50  0001 C CNN
F 1 "+5V" H 2265 3073 50  0000 C CNN
F 2 "" H 2250 2900 50  0001 C CNN
F 3 "" H 2250 2900 50  0001 C CNN
	1    2250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3600 2250 3600
Wire Wire Line
	2250 3600 2250 4000
Wire Wire Line
	2100 3500 2250 3500
Wire Wire Line
	2250 3500 2250 2900
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 5BBC5C1F
P 5750 3500
F 0 "J2" H 5830 3542 50  0000 L CNN
F 1 "Conn_01x05" H 5830 3451 50  0000 L CNN
F 2 "" H 5750 3500 50  0001 C CNN
F 3 "~" H 5750 3500 50  0001 C CNN
	1    5750 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5BBC5C5E
P 5150 4050
F 0 "#PWR04" H 5150 3800 50  0001 C CNN
F 1 "GND" H 5155 3877 50  0000 C CNN
F 2 "" H 5150 4050 50  0001 C CNN
F 3 "" H 5150 4050 50  0001 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 5BBC5C6D
P 5450 3100
F 0 "#PWR02" H 5450 2950 50  0001 C CNN
F 1 "+5V" H 5465 3273 50  0000 C CNN
F 2 "" H 5450 3100 50  0001 C CNN
F 3 "" H 5450 3100 50  0001 C CNN
	1    5450 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3100 5450 3300
Wire Wire Line
	5450 3300 5550 3300
Wire Wire Line
	2100 3400 5550 3400
$Comp
L Connector:Conn_01x03_Female J3
U 1 1 5BBC5E11
P 1900 4750
F 0 "J3" H 1794 5035 50  0000 C CNN
F 1 "Conn_01x03_Female" H 1794 4944 50  0000 C CNN
F 2 "" H 1900 4750 50  0001 C CNN
F 3 "~" H 1900 4750 50  0001 C CNN
	1    1900 4750
	-1   0    0    -1  
$EndComp
Text Notes 750  3550 0    50   ~ 0
To light barrier PCB
Text Notes 750  4800 0    50   ~ 0
To SHARP distance sensor\n(GP2Y0A21YK0F)
Wire Wire Line
	4550 4650 4550 3600
Wire Wire Line
	4550 3600 5150 3600
$Comp
L Device:R R1
U 1 1 5BBC5FF2
P 2500 4650
F 0 "R1" V 2293 4650 50  0000 C CNN
F 1 "3k3" V 2384 4650 50  0000 C CNN
F 2 "" V 2430 4650 50  0001 C CNN
F 3 "~" H 2500 4650 50  0001 C CNN
	1    2500 4650
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 4650 2100 4650
Wire Wire Line
	2650 4650 4550 4650
$Comp
L Device:CP C3
U 1 1 5BBC61D0
P 2850 5050
F 0 "C3" H 2968 5096 50  0000 L CNN
F 1 "22u" H 2968 5005 50  0000 L CNN
F 2 "" H 2888 4900 50  0001 C CNN
F 3 "~" H 2850 5050 50  0001 C CNN
	1    2850 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5BBC6244
P 2450 5050
F 0 "C2" H 2565 5096 50  0000 L CNN
F 1 "100n" H 2565 5005 50  0000 L CNN
F 2 "" H 2488 4900 50  0001 C CNN
F 3 "~" H 2450 5050 50  0001 C CNN
	1    2450 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5BBC62D0
P 5150 3800
F 0 "C1" H 5265 3846 50  0000 L CNN
F 1 "100n" H 5265 3755 50  0000 L CNN
F 2 "" H 5188 3650 50  0001 C CNN
F 3 "~" H 5150 3800 50  0001 C CNN
	1    5150 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5BBC6354
P 2450 5300
F 0 "#PWR07" H 2450 5050 50  0001 C CNN
F 1 "GND" H 2455 5127 50  0000 C CNN
F 2 "" H 2450 5300 50  0001 C CNN
F 3 "" H 2450 5300 50  0001 C CNN
	1    2450 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5BBC636D
P 2850 5300
F 0 "#PWR08" H 2850 5050 50  0001 C CNN
F 1 "GND" H 2855 5127 50  0000 C CNN
F 2 "" H 2850 5300 50  0001 C CNN
F 3 "" H 2850 5300 50  0001 C CNN
	1    2850 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BBC6386
P 2250 5300
F 0 "#PWR06" H 2250 5050 50  0001 C CNN
F 1 "GND" H 2255 5127 50  0000 C CNN
F 2 "" H 2250 5300 50  0001 C CNN
F 3 "" H 2250 5300 50  0001 C CNN
	1    2250 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 4850 2450 4850
Wire Wire Line
	2450 4850 2450 4900
Wire Wire Line
	2450 4850 2850 4850
Wire Wire Line
	2850 4850 2850 4900
Connection ~ 2450 4850
Wire Wire Line
	2450 5200 2450 5300
Wire Wire Line
	2850 5300 2850 5200
Wire Wire Line
	2100 4750 2250 4750
Wire Wire Line
	2250 4750 2250 5300
Wire Wire Line
	5150 3600 5150 3650
Connection ~ 5150 3600
Wire Wire Line
	5150 3600 5550 3600
$Comp
L power:GND #PWR05
U 1 1 5BBC7143
P 5500 4050
F 0 "#PWR05" H 5500 3800 50  0001 C CNN
F 1 "GND" H 5505 3877 50  0000 C CNN
F 2 "" H 5500 4050 50  0001 C CNN
F 3 "" H 5500 4050 50  0001 C CNN
	1    5500 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3950 5150 4050
Wire Wire Line
	5550 3500 5500 3500
Wire Wire Line
	5500 3500 5500 3700
Wire Wire Line
	5550 3700 5500 3700
Connection ~ 5500 3700
Wire Wire Line
	5500 3700 5500 4050
Text Notes 7150 6950 0    100  ~ 0
Interface PCB between MCU and sensors\n(light barrier / distance)
$EndSCHEMATC
