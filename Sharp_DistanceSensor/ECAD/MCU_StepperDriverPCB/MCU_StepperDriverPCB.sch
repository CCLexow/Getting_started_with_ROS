EESchema Schematic File Version 4
LIBS:MCU_StepperDriverPCB-cache
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
L Driver_Motor:Pololu_Breakout_A4988 A1
U 1 1 5BBEEFB7
P 8650 2800
F 0 "A1" H 8700 3678 50  0000 C CNN
F 1 "Watterott SilentStepStick TMC2100" H 7850 3500 50  0000 C CNN
F 2 "Module:Pololu_Breakout-16_15.2x20.3mm" H 8925 2050 50  0001 L CNN
F 3 "https://www.pololu.com/product/2980/pictures" H 8750 2500 50  0001 C CNN
	1    8650 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:D D2
U 1 1 5BBEF22D
P 2550 1250
F 0 "D2" H 2550 1034 50  0000 C CNN
F 1 "NRVTS260ESFT1G" H 2550 1125 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123F" H 2550 1250 50  0001 C CNN
F 3 "~" H 2550 1250 50  0001 C CNN
	1    2550 1250
	-1   0    0    1   
$EndComp
$Comp
L Device:D D1
U 1 1 5BBEF2A9
P 3700 800
F 0 "D1" H 3700 1016 50  0000 C CNN
F 1 "NRVTS260ESFT1G" H 3700 925 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123F" H 3700 800 50  0001 C CNN
F 3 "~" H 3700 800 50  0001 C CNN
	1    3700 800 
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5BBEF3CF
P 1550 1350
F 0 "J1" H 1470 1025 50  0000 C CNN
F 1 "Conn_01x02" H 1470 1116 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_2x01_P2.00mm_Vertical" H 1550 1350 50  0001 C CNN
F 3 "~" H 1550 1350 50  0001 C CNN
	1    1550 1350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5BBEF590
P 2200 1650
F 0 "#PWR07" H 2200 1400 50  0001 C CNN
F 1 "GND" H 2205 1477 50  0000 C CNN
F 2 "" H 2200 1650 50  0001 C CNN
F 3 "" H 2200 1650 50  0001 C CNN
	1    2200 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5BBEF5AC
P 8650 3750
F 0 "#PWR017" H 8650 3500 50  0001 C CNN
F 1 "GND" H 8655 3577 50  0000 C CNN
F 2 "" H 8650 3750 50  0001 C CNN
F 3 "" H 8650 3750 50  0001 C CNN
	1    8650 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5BBEF5C1
P 8850 3750
F 0 "#PWR018" H 8850 3500 50  0001 C CNN
F 1 "GND" H 8855 3577 50  0000 C CNN
F 2 "" H 8850 3750 50  0001 C CNN
F 3 "" H 8850 3750 50  0001 C CNN
	1    8850 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5BBEF5D6
P 3700 1650
F 0 "#PWR04" H 3700 1400 50  0001 C CNN
F 1 "GND" H 3705 1477 50  0000 C CNN
F 2 "" H 3700 1650 50  0001 C CNN
F 3 "" H 3700 1650 50  0001 C CNN
	1    3700 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5BBEF5EB
P 3200 1650
F 0 "#PWR03" H 3200 1400 50  0001 C CNN
F 1 "GND" H 3205 1477 50  0000 C CNN
F 2 "" H 3200 1650 50  0001 C CNN
F 3 "" H 3200 1650 50  0001 C CNN
	1    3200 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5BBEF600
P 4200 1650
F 0 "#PWR05" H 4200 1400 50  0001 C CNN
F 1 "GND" H 4205 1477 50  0000 C CNN
F 2 "" H 4200 1650 50  0001 C CNN
F 3 "" H 4200 1650 50  0001 C CNN
	1    4200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5BBEF6C5
P 2200 1450
F 0 "C1" H 2318 1496 50  0000 L CNN
F 1 "470u/25V" H 2318 1405 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P5.00mm" H 2238 1300 50  0001 C CNN
F 3 "~" H 2200 1450 50  0001 C CNN
	1    2200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR02
U 1 1 5BBEF793
P 2200 1200
F 0 "#PWR02" H 2200 1050 50  0001 C CNN
F 1 "+12V" H 2215 1373 50  0000 C CNN
F 2 "" H 2200 1200 50  0001 C CNN
F 3 "" H 2200 1200 50  0001 C CNN
	1    2200 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BBEF7F8
P 1850 1650
F 0 "#PWR06" H 1850 1400 50  0001 C CNN
F 1 "GND" H 1855 1477 50  0000 C CNN
F 2 "" H 1850 1650 50  0001 C CNN
F 3 "" H 1850 1650 50  0001 C CNN
	1    1850 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1350 1850 1350
Wire Wire Line
	1850 1350 1850 1650
Wire Wire Line
	1750 1250 2200 1250
Wire Wire Line
	2200 1250 2200 1200
Wire Wire Line
	2200 1300 2200 1250
Connection ~ 2200 1250
Wire Wire Line
	2200 1600 2200 1650
$Comp
L Device:C C2
U 1 1 5BBEF91E
P 3200 1450
F 0 "C2" H 3315 1496 50  0000 L CNN
F 1 "1u" H 3315 1405 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3238 1300 50  0001 C CNN
F 3 "~" H 3200 1450 50  0001 C CNN
	1    3200 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5BBEF95D
P 4200 1450
F 0 "C3" H 4315 1496 50  0000 L CNN
F 1 "100n" H 4315 1405 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4238 1300 50  0001 C CNN
F 3 "~" H 4200 1450 50  0001 C CNN
	1    4200 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1600 3200 1650
$Comp
L Device:D D3
U 1 1 5BBEFAE1
P 9350 2500
F 0 "D3" V 9304 2579 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 8850 2400 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 9350 2500 50  0001 C CNN
F 3 "~" H 9350 2500 50  0001 C CNN
	1    9350 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D D8
U 1 1 5BBEFBD0
P 9350 3200
F 0 "D8" V 9304 3279 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9250 3100 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 9350 3200 50  0001 C CNN
F 3 "~" H 9350 3200 50  0001 C CNN
	1    9350 3200
	0    1    1    0   
$EndComp
$Comp
L Device:D D4
U 1 1 5BBF00D7
P 9700 2500
F 0 "D4" V 9654 2579 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9200 2400 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 9700 2500 50  0001 C CNN
F 3 "~" H 9700 2500 50  0001 C CNN
	1    9700 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D D9
U 1 1 5BBF00DE
P 9700 3200
F 0 "D9" V 9654 3279 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9600 3100 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 9700 3200 50  0001 C CNN
F 3 "~" H 9700 3200 50  0001 C CNN
	1    9700 3200
	0    1    1    0   
$EndComp
$Comp
L Device:D D5
U 1 1 5BBF0125
P 10050 2500
F 0 "D5" V 10004 2579 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9550 2400 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 10050 2500 50  0001 C CNN
F 3 "~" H 10050 2500 50  0001 C CNN
	1    10050 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D D10
U 1 1 5BBF012C
P 10050 3200
F 0 "D10" V 10004 3279 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9950 3100 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 10050 3200 50  0001 C CNN
F 3 "~" H 10050 3200 50  0001 C CNN
	1    10050 3200
	0    1    1    0   
$EndComp
$Comp
L Device:D D6
U 1 1 5BBF0179
P 10400 2500
F 0 "D6" V 10354 2579 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 9900 2400 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 10400 2500 50  0001 C CNN
F 3 "~" H 10400 2500 50  0001 C CNN
	1    10400 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D D11
U 1 1 5BBF0180
P 10400 3200
F 0 "D11" V 10354 3279 50  0000 L CNN
F 1 "NRVTS260ESFT1G" H 10300 3100 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123F" H 10400 3200 50  0001 C CNN
F 3 "~" H 10400 3200 50  0001 C CNN
	1    10400 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5BBF01B9
P 9350 3450
F 0 "#PWR012" H 9350 3200 50  0001 C CNN
F 1 "GND" H 9355 3277 50  0000 C CNN
F 2 "" H 9350 3450 50  0001 C CNN
F 3 "" H 9350 3450 50  0001 C CNN
	1    9350 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5BBF01E4
P 9700 3450
F 0 "#PWR013" H 9700 3200 50  0001 C CNN
F 1 "GND" H 9705 3277 50  0000 C CNN
F 2 "" H 9700 3450 50  0001 C CNN
F 3 "" H 9700 3450 50  0001 C CNN
	1    9700 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5BBF020F
P 10050 3450
F 0 "#PWR014" H 10050 3200 50  0001 C CNN
F 1 "GND" H 10055 3277 50  0000 C CNN
F 2 "" H 10050 3450 50  0001 C CNN
F 3 "" H 10050 3450 50  0001 C CNN
	1    10050 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5BBF023A
P 10400 3450
F 0 "#PWR015" H 10400 3200 50  0001 C CNN
F 1 "GND" H 10405 3277 50  0000 C CNN
F 2 "" H 10400 3450 50  0001 C CNN
F 3 "" H 10400 3450 50  0001 C CNN
	1    10400 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 2700 9350 2700
Wire Wire Line
	9150 2800 9700 2800
Wire Wire Line
	9150 2900 10050 2900
Wire Wire Line
	9150 3000 10400 3000
Wire Wire Line
	9350 3350 9350 3450
Wire Wire Line
	9700 3350 9700 3450
Wire Wire Line
	10050 3350 10050 3450
Wire Wire Line
	10400 3350 10400 3450
Wire Wire Line
	8850 2100 8850 1800
Wire Wire Line
	8850 1800 9350 1800
Wire Wire Line
	10400 1800 10400 2350
Wire Wire Line
	10050 2350 10050 1800
Connection ~ 10050 1800
Wire Wire Line
	10050 1800 10400 1800
Wire Wire Line
	9700 2350 9700 1800
Connection ~ 9700 1800
Wire Wire Line
	9700 1800 10050 1800
Wire Wire Line
	9350 2350 9350 1800
Connection ~ 9350 1800
Wire Wire Line
	9350 1800 9700 1800
$Comp
L power:+12V #PWR08
U 1 1 5BBF0D3E
P 8850 1700
F 0 "#PWR08" H 8850 1550 50  0001 C CNN
F 1 "+12V" H 8865 1873 50  0000 C CNN
F 2 "" H 8850 1700 50  0001 C CNN
F 3 "" H 8850 1700 50  0001 C CNN
	1    8850 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1700 8850 1800
Connection ~ 8850 1800
Wire Wire Line
	9350 3050 9350 2700
Connection ~ 9350 2700
Wire Wire Line
	9350 2700 10650 2700
Wire Wire Line
	9350 2650 9350 2700
Wire Wire Line
	9700 2650 9700 2800
Connection ~ 9700 2800
Wire Wire Line
	9700 2800 10650 2800
Wire Wire Line
	9700 2800 9700 3050
Wire Wire Line
	10050 3050 10050 2900
Connection ~ 10050 2900
Wire Wire Line
	10050 2900 10650 2900
Wire Wire Line
	10050 2900 10050 2650
Wire Wire Line
	10400 2650 10400 3000
Connection ~ 10400 3000
Wire Wire Line
	10400 3000 10650 3000
Wire Wire Line
	10400 3000 10400 3050
Wire Wire Line
	8650 3600 8650 3750
Wire Wire Line
	8850 3600 8850 3750
NoConn ~ 8250 2500
NoConn ~ 8250 2400
$Comp
L power:+5V #PWR01
U 1 1 5BBF486F
P 4200 1150
F 0 "#PWR01" H 4200 1000 50  0001 C CNN
F 1 "+5V" H 4215 1323 50  0000 C CNN
F 2 "" H 4200 1150 50  0001 C CNN
F 3 "" H 4200 1150 50  0001 C CNN
	1    4200 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR09
U 1 1 5BBF48A3
P 8650 1850
F 0 "#PWR09" H 8650 1700 50  0001 C CNN
F 1 "+5V" H 8665 2023 50  0000 C CNN
F 2 "" H 8650 1850 50  0001 C CNN
F 3 "" H 8650 1850 50  0001 C CNN
	1    8650 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1850 8650 2100
Wire Wire Line
	3700 1550 3700 1650
Wire Wire Line
	2700 1250 2850 1250
Wire Wire Line
	3200 1300 3200 1250
Connection ~ 3200 1250
Wire Wire Line
	3200 1250 3400 1250
Wire Wire Line
	4000 1250 4050 1250
Wire Wire Line
	4200 1250 4200 1150
Wire Wire Line
	4200 1250 4200 1300
Connection ~ 4200 1250
Wire Wire Line
	4200 1650 4200 1600
Wire Wire Line
	4050 1250 4050 800 
Wire Wire Line
	4050 800  3850 800 
Connection ~ 4050 1250
Wire Wire Line
	4050 1250 4200 1250
Wire Wire Line
	3550 800  3200 800 
Wire Wire Line
	3200 800  3200 1250
$Comp
L STM32_BluePill:STM32_BluePill_PortView U2
U 1 1 5BC00245
P 2350 5100
F 0 "U2" H 2350 7087 60  0000 C CNN
F 1 "STM32_BluePill_PortView" H 2350 6981 60  0000 C CNN
F 2 "STM32_BluePill:STM32_BluePill_ROS_GS" H 4850 8250 60  0001 C CNN
F 3 "" H 4850 8250 60  0001 C CNN
	1    2350 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1250 2200 1250
$Comp
L Device:C C6
U 1 1 5BC0203D
P 1200 3750
F 0 "C6" H 1315 3796 50  0000 L CNN
F 1 "100n" H 1315 3705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1238 3600 50  0001 C CNN
F 3 "~" H 1200 3750 50  0001 C CNN
	1    1200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5BC020FB
P 3350 3600
F 0 "C5" H 3465 3646 50  0000 L CNN
F 1 "100n" H 3465 3555 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3388 3450 50  0001 C CNN
F 3 "~" H 3350 3600 50  0001 C CNN
	1    3350 3600
	1    0    0    -1  
$EndComp
NoConn ~ 1750 5050
NoConn ~ 1750 3400
$Comp
L Device:D D12
U 1 1 5BC04A6C
P 1200 3350
F 0 "D12" H 1200 3134 50  0000 C CNN
F 1 "NRVTS260ESFT1G" H 1200 3225 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123F" H 1200 3350 50  0001 C CNN
F 3 "~" H 1200 3350 50  0001 C CNN
	1    1200 3350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 3550 1200 3550
Wire Wire Line
	1200 3550 1200 3500
Wire Wire Line
	1200 3550 1200 3600
Connection ~ 1200 3550
$Comp
L power:+5V #PWR011
U 1 1 5BC069A1
P 1200 3150
F 0 "#PWR011" H 1200 3000 50  0001 C CNN
F 1 "+5V" H 1215 3323 50  0000 C CNN
F 2 "" H 1200 3150 50  0001 C CNN
F 3 "" H 1200 3150 50  0001 C CNN
	1    1200 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3200 1200 3150
$Comp
L power:GND #PWR020
U 1 1 5BC079C3
P 1200 3950
F 0 "#PWR020" H 1200 3700 50  0001 C CNN
F 1 "GND" H 1205 3777 50  0000 C CNN
F 2 "" H 1200 3950 50  0001 C CNN
F 3 "" H 1200 3950 50  0001 C CNN
	1    1200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3900 1200 3950
NoConn ~ 2950 3500
NoConn ~ 2950 3600
NoConn ~ 2950 3700
NoConn ~ 2950 3800
NoConn ~ 2950 3900
NoConn ~ 2950 4000
NoConn ~ 2950 4100
NoConn ~ 2950 4300
NoConn ~ 2950 4400
NoConn ~ 2950 4500
NoConn ~ 2950 4600
NoConn ~ 2950 4700
$Comp
L power:GND #PWR019
U 1 1 5BC18F90
P 3350 3850
F 0 "#PWR019" H 3350 3600 50  0001 C CNN
F 1 "GND" H 3355 3677 50  0000 C CNN
F 2 "" H 3350 3850 50  0001 C CNN
F 3 "" H 3350 3850 50  0001 C CNN
	1    3350 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3850 3350 3750
NoConn ~ 1750 3650
NoConn ~ 1750 3750
$Comp
L power:GND #PWR021
U 1 1 5BC1C652
P 1700 4300
F 0 "#PWR021" H 1700 4050 50  0001 C CNN
F 1 "GND" H 1705 4127 50  0000 C CNN
F 2 "" H 1700 4300 50  0001 C CNN
F 3 "" H 1700 4300 50  0001 C CNN
	1    1700 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4050 1700 4050
Wire Wire Line
	1700 4050 1700 4150
Wire Wire Line
	1750 4150 1700 4150
Connection ~ 1700 4150
Wire Wire Line
	1700 4150 1700 4250
NoConn ~ 2950 6600
NoConn ~ 2950 6700
NoConn ~ 2950 6800
Wire Wire Line
	7450 2800 8250 2800
Wire Wire Line
	7550 2900 8250 2900
Wire Wire Line
	7300 2700 8250 2700
Wire Wire Line
	2950 6300 7450 6300
Wire Wire Line
	7450 6300 7450 2800
Wire Wire Line
	2950 6200 7550 6200
Wire Wire Line
	7550 6200 7550 2900
NoConn ~ 8250 3300
NoConn ~ 8250 3200
$Comp
L Diode:1N4148WS D7
U 1 1 5BC31124
P 8000 3100
F 0 "D7" H 8000 2883 50  0000 C CNN
F 1 "1N4148WS" H 8000 2974 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-323" H 8000 2925 50  0001 C CNN
F 3 "https://www.vishay.com/docs/85751/1n4148ws.pdf" H 8000 3100 50  0001 C CNN
	1    8000 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	8150 3100 8250 3100
Wire Wire Line
	7700 3100 7850 3100
Text Label 3000 4200 0    50   ~ 0
STMD_En
Text Label 3000 6300 0    50   ~ 0
STMD_Step
Text Label 3000 6200 0    50   ~ 0
STMD_Dir
Text Label 3000 5700 0    50   ~ 0
Light_Barrier
Wire Wire Line
	2950 3400 3350 3400
Wire Wire Line
	3350 3400 3350 3450
NoConn ~ 2950 4900
NoConn ~ 2950 5000
NoConn ~ 2950 5300
NoConn ~ 2950 5400
NoConn ~ 2950 5500
NoConn ~ 2950 5600
NoConn ~ 2950 5800
NoConn ~ 2950 5900
NoConn ~ 2950 6000
NoConn ~ 2950 6100
$Comp
L Connector:Conn_01x05_Female J4
U 1 1 5BC501A7
P 5900 4100
F 0 "J4" H 5927 4126 50  0000 L CNN
F 1 "Conn_01x05_Female" H 5927 4035 50  0000 L CNN
F 2 "MicroMatch:MM-FL-6S" H 5900 4100 50  0001 C CNN
F 3 "~" H 5900 4100 50  0001 C CNN
	1    5900 4100
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5BC55F45
P 5800 3500
F 0 "#PWR016" H 5800 3250 50  0001 C CNN
F 1 "GND" H 5805 3327 50  0000 C CNN
F 2 "" H 5800 3500 50  0001 C CNN
F 3 "" H 5800 3500 50  0001 C CNN
	1    5800 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5BC55F4B
P 5800 3300
F 0 "C4" H 5915 3346 50  0000 L CNN
F 1 "100n" H 5915 3255 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5838 3150 50  0001 C CNN
F 3 "~" H 5800 3300 50  0001 C CNN
	1    5800 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 5BC55F52
P 5800 3050
F 0 "#PWR010" H 5800 2900 50  0001 C CNN
F 1 "+5V" H 5815 3223 50  0000 C CNN
F 2 "" H 5800 3050 50  0001 C CNN
F 3 "" H 5800 3050 50  0001 C CNN
	1    5800 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3500 5800 3450
Wire Wire Line
	5800 3050 5800 3100
Wire Wire Line
	5800 3100 5550 3100
Wire Wire Line
	5550 3100 5550 3900
Wire Wire Line
	5550 3900 5700 3900
Connection ~ 5800 3100
Wire Wire Line
	5800 3100 5800 3150
$Comp
L power:GND #PWR022
U 1 1 5BC60503
P 5550 4450
F 0 "#PWR022" H 5550 4200 50  0001 C CNN
F 1 "GND" H 5555 4277 50  0000 C CNN
F 2 "" H 5550 4450 50  0001 C CNN
F 3 "" H 5550 4450 50  0001 C CNN
	1    5550 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4100 5550 4100
Wire Wire Line
	5550 4100 5550 4300
Wire Wire Line
	5700 4300 5550 4300
Connection ~ 5550 4300
Wire Wire Line
	5550 4300 5550 4450
Wire Wire Line
	3350 3400 5300 3400
Wire Wire Line
	5300 3400 5300 4200
Wire Wire Line
	5300 4200 5700 4200
Connection ~ 3350 3400
Wire Wire Line
	4200 5700 4200 4000
Wire Wire Line
	4200 4000 5700 4000
Wire Wire Line
	2950 5700 4200 5700
Text Label 3050 3400 0    50   ~ 0
AN_Distance
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5BBF7C0A
P 10850 2800
F 0 "J2" H 10743 2475 50  0000 C CNN
F 1 "Conn_01x02_Female" H 10743 2566 50  0000 C CNN
F 2 "Connectors:MPT_0,5_02_2,54" H 10850 2800 50  0001 C CNN
F 3 "~" H 10850 2800 50  0001 C CNN
	1    10850 2800
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Female J3
U 1 1 5BBF7CBE
P 10850 3000
F 0 "J3" H 10743 2675 50  0000 C CNN
F 1 "Conn_01x02_Female" H 10743 2766 50  0000 C CNN
F 2 "Connectors:MPT_0,5_02_2,54" H 10850 3000 50  0001 C CNN
F 3 "~" H 10850 3000 50  0001 C CNN
	1    10850 3000
	1    0    0    1   
$EndComp
$Comp
L Device:CP C7
U 1 1 5BC1B577
P 2850 1450
F 0 "C7" H 2968 1496 50  0000 L CNN
F 1 "4u7/35V" H 2968 1405 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_4x4.5" H 2888 1300 50  0001 C CNN
F 3 "~" H 2850 1450 50  0001 C CNN
	1    2850 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5BC1B5CA
P 2850 1650
F 0 "#PWR023" H 2850 1400 50  0001 C CNN
F 1 "GND" H 2855 1477 50  0000 C CNN
F 2 "" H 2850 1650 50  0001 C CNN
F 3 "" H 2850 1650 50  0001 C CNN
	1    2850 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1650 2850 1600
Wire Wire Line
	2850 1300 2850 1250
Connection ~ 2850 1250
Wire Wire Line
	2850 1250 3200 1250
Wire Wire Line
	1750 4250 1700 4250
Connection ~ 1700 4250
Wire Wire Line
	1700 4250 1700 4300
Wire Wire Line
	2950 4200 4850 4200
Wire Wire Line
	4850 4200 4850 5000
Wire Wire Line
	4850 5000 7300 5000
Wire Wire Line
	7300 5000 7300 2700
Wire Wire Line
	2950 5200 7700 5200
Wire Wire Line
	7700 3100 7700 5200
NoConn ~ 2950 5100
$Comp
L Toshiba~-~Power:TA78L05F U1
U 1 1 5BC4AD05
P 3700 1250
F 0 "U1" H 3700 1492 50  0000 C CNN
F 1 "TA78L05F" H 3700 1401 50  0000 C CNN
F 2 "SMD_Packages_Millable:SOT-89-3_millable_0.6mm" H 3725 1100 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 3700 1200 50  0001 C CNN
	1    3700 1250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
