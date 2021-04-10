EESchema Schematic File Version 4
EELAYER 30 0
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
L MCU_Module:Arduino_Nano_v3.x A?
U 1 1 6052BBF2
P 5400 3300
F 0 "A?" H 5400 2211 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5400 2120 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5400 3300 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5400 3300 50  0001 C CNN
	1    5400 3300
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1117-5.0 U?
U 1 1 60538858
P 3400 2200
F 0 "U?" H 3400 2442 50  0000 C CNN
F 1 "LM1117-5.0" H 3400 2351 50  0000 C CNN
F 2 "" H 3400 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 3400 2200 50  0001 C CNN
	1    3400 2200
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1117-3.3 U?
U 1 1 60539332
P 3400 3050
F 0 "U?" H 3400 3292 50  0000 C CNN
F 1 "LM1117-3.3" H 3400 3201 50  0000 C CNN
F 2 "" H 3400 3050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 3400 3050 50  0001 C CNN
	1    3400 3050
	1    0    0    -1  
$EndComp
$Comp
L spinnybot:SN74AHCT541N U?
U 1 1 6052D474
P 5150 1150
F 0 "U?" H 5175 1831 50  0000 C CNN
F 1 "SN74AHCT541N" H 5175 1740 50  0000 C CNN
F 2 "" H 5000 1200 50  0001 C CNN
F 3 "" H 5000 1200 50  0001 C CNN
	1    5150 1150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H?
U 1 1 60541153
P 2750 1400
F 0 "H?" V 2987 1403 50  0000 C CNN
F 1 "GND" V 2896 1403 50  0000 C CNN
F 2 "" H 2750 1400 50  0001 C CNN
F 3 "~" H 2750 1400 50  0001 C CNN
	1    2750 1400
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H?
U 1 1 605409FF
P 2750 950
F 0 "H?" V 2987 953 50  0000 C CNN
F 1 "+12V" V 2896 953 50  0000 C CNN
F 2 "" H 2750 950 50  0001 C CNN
F 3 "~" H 2750 950 50  0001 C CNN
	1    2750 950 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 950  3100 950 
Wire Wire Line
	3100 950  3100 2200
Wire Wire Line
	3100 2200 3100 3050
Connection ~ 3100 2200
Wire Wire Line
	2850 1400 2850 2500
Wire Wire Line
	2850 2500 3400 2500
Wire Wire Line
	2850 2500 2850 3350
Wire Wire Line
	2850 3350 3400 3350
Connection ~ 2850 2500
$EndSCHEMATC
