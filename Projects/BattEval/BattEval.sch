EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A2 23386 16535
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
L SamacSys_Parts:ATSAMD21G18A-MU IC4
U 1 1 6045EE1A
P 15000 10000
F 0 "IC4" H 15750 11100 50  0000 L CNN
F 1 "ATSAMD21G18A-MU" H 15750 11000 50  0000 L CNN
F 2 "QFN50P700X700X90-49N-D" H 14450 8000 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf" H 14450 7900 50  0001 L CNN
F 4 "Microchip Technology ATSAMD21G18A-MU, 32bit Microcontroller, 48MHz, 256 kB Flash, 48-Pin QFN" H 14450 7800 50  0001 L CNN "Description"
F 5 "0.9" H 14450 7700 50  0001 L CNN "Height"
F 6 "556-ATSAMD21G18A-MU" H 14450 7600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Microchip-Technology/ATSAMD21G18A-MU/?qs=KLFHFgXTQiA%2FfMu%2FGM5u1A%3D%3D" H 14450 7500 50  0001 L CNN "Mouser Price/Stock"
F 8 "Microchip" H 14450 7400 50  0001 L CNN "Manufacturer_Name"
F 9 "ATSAMD21G18A-MU" H 14450 7300 50  0001 L CNN "Manufacturer_Part_Number"
	1    15000 10000
	1    0    0    -1  
$EndComp
Text GLabel 15900 9450 2    50   Input ~ 0
3v3
Text GLabel 15900 9850 2    50   BiDi ~ 0
D21
Text GLabel 15900 9950 2    50   BiDi ~ 0
D20
Text GLabel 15900 10150 2    50   BiDi ~ 0
D6
Text GLabel 15900 10250 2    50   BiDi ~ 0
D12
Text GLabel 15900 10350 2    50   BiDi ~ 0
D10
Text GLabel 15900 10550 2    50   BiDi ~ 0
D11
Text GLabel 15500 11350 3    50   BiDi ~ 0
D5
Text GLabel 14700 11350 3    50   Input ~ 0
DATA_RX
Text GLabel 14600 11350 3    50   Output ~ 0
DATA_TX
Text GLabel 14100 10550 0    50   BiDi ~ 0
D9
Text GLabel 11650 10150 0    50   Input ~ 0
BATT
$Comp
L power:GNDREF #PWR018
U 1 1 604698FB
P 14900 11550
F 0 "#PWR018" H 14900 11300 50  0001 C CNN
F 1 "GNDREF" H 14905 11377 50  0000 C CNN
F 2 "" H 14900 11550 50  0001 C CNN
F 3 "" H 14900 11550 50  0001 C CNN
	1    14900 11550
	1    0    0    -1  
$EndComp
Wire Wire Line
	14900 11550 14900 11350
$Comp
L power:GNDREF #PWR024
U 1 1 6046BB0B
P 15900 9550
F 0 "#PWR024" H 15900 9300 50  0001 C CNN
F 1 "GNDREF" V 15905 9422 50  0000 R CNN
F 2 "" H 15900 9550 50  0001 C CNN
F 3 "" H 15900 9550 50  0001 C CNN
	1    15900 9550
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR017
U 1 1 6046CD33
P 14400 8650
F 0 "#PWR017" H 14400 8400 50  0001 C CNN
F 1 "GNDREF" H 14405 8477 50  0000 C CNN
F 2 "" H 14400 8650 50  0001 C CNN
F 3 "" H 14400 8650 50  0001 C CNN
	1    14400 8650
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR019
U 1 1 6046CE66
P 15100 8150
F 0 "#PWR019" H 15100 7900 50  0001 C CNN
F 1 "GNDREF" H 15105 7977 50  0000 C CNN
F 2 "" H 15100 8150 50  0001 C CNN
F 3 "" H 15100 8150 50  0001 C CNN
	1    15100 8150
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:06033C104KAT2A C9
U 1 1 6046D52E
P 16150 9350
F 0 "C9" H 16150 9615 50  0000 C CNN
F 1 "06033C104KAT2A" H 15850 8750 50  0001 L CNN
F 2 "CAPC1608X90N" H 15850 8850 50  0001 L CNN
F 3 "" H 15850 8750 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 15850 8650 50  0001 L CNN "Description"
F 5 "0.9" H 15850 8550 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 15850 8450 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 15850 8350 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 15850 8250 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 15850 8150 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" H 16150 9524 50  0000 C CNN "Capacitance"
	1    16150 9350
	1    0    0    -1  
$EndComp
Wire Wire Line
	15900 9350 15900 9450
$Comp
L power:GNDREF #PWR025
U 1 1 6046E09E
P 16400 9350
F 0 "#PWR025" H 16400 9100 50  0001 C CNN
F 1 "GNDREF" V 16405 9222 50  0000 R CNN
F 2 "" H 16400 9350 50  0001 C CNN
F 3 "" H 16400 9350 50  0001 C CNN
	1    16400 9350
	0    -1   -1   0   
$EndComp
NoConn ~ 15900 10050
NoConn ~ 15400 11350
NoConn ~ 15300 11350
NoConn ~ 14500 11350
NoConn ~ 14100 10450
NoConn ~ 14500 8650
NoConn ~ 15600 8650
NoConn ~ 15500 8650
NoConn ~ 15400 8650
Text GLabel 14900 8650 1    50   Input ~ 0
3v3
Text GLabel 14800 8650 1    50   Input ~ 0
SWCLK
Text GLabel 14700 8650 1    50   BiDi ~ 0
SWDIO
$Comp
L SamacSys_Parts:0603ZC105KAT2A C7
U 1 1 6046F0DB
P 15000 8400
F 0 "C7" V 15046 8272 50  0000 R CNN
F 1 "0603ZC105KAT2A" H 14700 7700 50  0001 L CNN
F 2 "CAPC1608X90N" H 14700 7800 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/0603ZC105KAT2A.pdf" H 14700 7700 50  0001 L CNN
F 4 "AVX - 0603ZC105KAT2A - CAP, MLCC, X7R, 1UF, 10V, 0603" H 14700 7600 50  0001 L CNN "Description"
F 5 "0.9" H 14700 7500 50  0001 L CNN "Height"
F 6 "581-0603ZC105KAT2A" H 14700 7400 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=581-0603ZC105KAT2A" H 14700 7300 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 14700 7200 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZC105KAT2A" H 14700 7100 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "1 uF" V 14955 8272 50  0000 R CNN "Capacitance"
	1    15000 8400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	15100 8650 15100 8150
Wire Wire Line
	15100 8150 15000 8150
Connection ~ 15100 8150
$Comp
L SamacSys_Parts:06033C104KAT2A C6
U 1 1 60471EC7
P 14550 12050
F 0 "C6" H 14550 12315 50  0000 C CNN
F 1 "06033C104KAT2A" H 14250 11450 50  0001 L CNN
F 2 "CAPC1608X90N" H 14250 11550 50  0001 L CNN
F 3 "" H 14250 11450 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 14250 11350 50  0001 L CNN "Description"
F 5 "0.9" H 14250 11250 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 14250 11150 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 14250 11050 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 14250 10950 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 14250 10850 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" H 14550 12224 50  0000 C CNN "Capacitance"
	1    14550 12050
	1    0    0    -1  
$EndComp
Wire Wire Line
	14800 12050 14800 11350
Text GLabel 14800 12050 3    50   Input ~ 0
3v3
$Comp
L power:GNDREF #PWR016
U 1 1 604740A0
P 14300 12050
F 0 "#PWR016" H 14300 11800 50  0001 C CNN
F 1 "GNDREF" V 14305 11922 50  0000 R CNN
F 2 "" H 14300 12050 50  0001 C CNN
F 3 "" H 14300 12050 50  0001 C CNN
	1    14300 12050
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR014
U 1 1 60474333
P 14100 9850
F 0 "#PWR014" H 14100 9600 50  0001 C CNN
F 1 "GNDREF" V 14105 9722 50  0000 R CNN
F 2 "" H 14100 9850 50  0001 C CNN
F 3 "" H 14100 9850 50  0001 C CNN
	1    14100 9850
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:06033C104KAT2A C2
U 1 1 60474820
P 12700 9700
F 0 "C2" V 12654 9828 50  0000 L CNN
F 1 "06033C104KAT2A" H 12400 9100 50  0001 L CNN
F 2 "CAPC1608X90N" H 12400 9200 50  0001 L CNN
F 3 "" H 12400 9100 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 12400 9000 50  0001 L CNN "Description"
F 5 "0.9" H 12400 8900 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 12400 8800 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 12400 8700 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 12400 8600 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 12400 8500 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" V 12745 9828 50  0000 L CNN "Capacitance"
	1    12700 9700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	14100 9950 12700 9950
Text GLabel 12700 9950 0    50   Input ~ 0
3v3
$Comp
L power:GNDREF #PWR08
U 1 1 6047C731
P 12700 9450
F 0 "#PWR08" H 12700 9200 50  0001 C CNN
F 1 "GNDREF" H 12705 9277 50  0000 C CNN
F 2 "" H 12700 9450 50  0001 C CNN
F 3 "" H 12700 9450 50  0001 C CNN
	1    12700 9450
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:06035A220JAT4A C5
U 1 1 604801CD
P 13900 9200
F 0 "C5" V 13946 9072 50  0000 R CNN
F 1 "06035A220JAT4A" H 13600 8600 50  0001 L CNN
F 2 "CAPC1608X90N" H 13600 8700 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06031A100FAT2A.pdf" H 13600 8600 50  0001 L CNN
F 4 "AVX - 06035A220JAT4A - CERAMIC CAPACITOR 22PF 50V, C0G, 5%, 0603, FULL REEL" H 13600 8500 50  0001 L CNN "Description"
F 5 "0.9" H 13600 8400 50  0001 L CNN "Height"
F 6 "581-06035A220JAT4A" H 13600 8300 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06035A220JAT4A/?qs=WmCxkUxNecQT6QFugq%2FgJA%3D%3D" H 13600 8200 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 13600 8100 50  0001 L CNN "Manufacturer_Name"
F 9 "06035A220JAT4A" H 13600 8000 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "22 pF" V 13855 9072 50  0000 R CNN "Capacitance"
	1    13900 9200
	0    -1   -1   0   
$EndComp
$Comp
L SamacSys_Parts:06035A220JAT4A C3
U 1 1 60481013
P 13200 9200
F 0 "C3" V 13250 9550 50  0000 R CNN
F 1 "06035A220JAT4A" H 12900 8600 50  0001 L CNN
F 2 "CAPC1608X90N" H 12900 8700 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06031A100FAT2A.pdf" H 12900 8600 50  0001 L CNN
F 4 "AVX - 06035A220JAT4A - CERAMIC CAPACITOR 22PF 50V, C0G, 5%, 0603, FULL REEL" H 12900 8500 50  0001 L CNN "Description"
F 5 "0.9" H 12900 8400 50  0001 L CNN "Height"
F 6 "581-06035A220JAT4A" H 12900 8300 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06035A220JAT4A/?qs=WmCxkUxNecQT6QFugq%2FgJA%3D%3D" H 12900 8200 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 12900 8100 50  0001 L CNN "Manufacturer_Name"
F 9 "06035A220JAT4A" H 12900 8000 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "22 pF" V 13150 9550 50  0000 R CNN "Capacitance"
	1    13200 9200
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR013
U 1 1 6048326B
P 13900 8950
F 0 "#PWR013" H 13900 8700 50  0001 C CNN
F 1 "GNDREF" H 13905 8777 50  0000 C CNN
F 2 "" H 13900 8950 50  0001 C CNN
F 3 "" H 13900 8950 50  0001 C CNN
	1    13900 8950
	-1   0    0    1   
$EndComp
Wire Wire Line
	14100 9450 13900 9450
Connection ~ 13900 9450
Wire Wire Line
	13900 9450 13600 9450
Wire Wire Line
	14100 9550 13500 9550
Wire Wire Line
	13500 9550 13500 9450
Wire Wire Line
	13500 9550 13200 9550
Wire Wire Line
	13200 9550 13200 9450
Connection ~ 13500 9550
$Comp
L power:GNDREF #PWR09
U 1 1 60484533
P 13200 8950
F 0 "#PWR09" H 13200 8700 50  0001 C CNN
F 1 "GNDREF" H 13205 8777 50  0000 C CNN
F 2 "" H 13200 8950 50  0001 C CNN
F 3 "" H 13200 8950 50  0001 C CNN
	1    13200 8950
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:CM7V-T1A-32.768kHz-6pF-20PPM-TA-QC Y1
U 1 1 6048599B
P 13550 9050
F 0 "Y1" H 13750 8850 50  0000 R CNN
F 1 "CM7V-T1A-32.768kHz-6pF-20PPM-TA-QC" H 12950 8450 50  0001 L CNN
F 2 "CM7VT1A32768kHz6pF20PPMTAQC" H 12950 8550 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/CM7V-T1A-32.768kHz-6pF-20PPM-TA-QC.pdf" H 12950 8450 50  0001 L CNN
F 4 "Crystals 32.768 kHz 6.0 pF +/-20 PPM -40/+85C" H 12950 8350 50  0001 L CNN "Description"
F 5 "0.65" H 12950 8250 50  0001 L CNN "Height"
F 6 "428-201843-MG01" H 12950 8150 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Micro-Crystal/CM7V-T1A-32768kHz-6pF-20PPM-TA-QC?qs=7bTaA%2FLYtSZ7ZoUG8vJXMg%3D%3D" H 12950 8050 50  0001 L CNN "Mouser Price/Stock"
F 8 "Micro Crystal AG" H 12950 7950 50  0001 L CNN "Manufacturer_Name"
F 9 "CM7V-T1A-32.768kHz-6pF-20PPM-TA-QC" H 12950 7850 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "32.768 kHz" H 13750 9250 50  0000 R CNN "Frequency"
	1    13550 9050
	0    -1   -1   0   
$EndComp
$Comp
L SamacSys_Parts:0603ZC105KAT2A C4
U 1 1 6048754D
P 13550 9750
F 0 "C4" H 13600 9600 50  0000 R CNN
F 1 "0603ZC105KAT2A" H 13250 9050 50  0001 L CNN
F 2 "CAPC1608X90N" H 13250 9150 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/0603ZC105KAT2A.pdf" H 13250 9050 50  0001 L CNN
F 4 "AVX - 0603ZC105KAT2A - CAP, MLCC, X7R, 1UF, 10V, 0603" H 13250 8950 50  0001 L CNN "Description"
F 5 "0.9" H 13250 8850 50  0001 L CNN "Height"
F 6 "581-0603ZC105KAT2A" H 13250 8750 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=581-0603ZC105KAT2A" H 13250 8650 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 13250 8550 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZC105KAT2A" H 13250 8450 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "1 uF" H 13650 9900 50  0000 R CNN "Capacitance"
	1    13550 9750
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR010
U 1 1 60488E4D
P 13300 9750
F 0 "#PWR010" H 13300 9500 50  0001 C CNN
F 1 "GNDREF" V 13305 9622 50  0000 R CNN
F 2 "" H 13300 9750 50  0001 C CNN
F 3 "" H 13300 9750 50  0001 C CNN
	1    13300 9750
	0    1    1    0   
$EndComp
Wire Wire Line
	14100 9750 13800 9750
$Comp
L SamacSys_Parts:47346-0001 J3
U 1 1 6048FC9D
P 18550 9750
F 0 "J3" H 18750 10100 50  0000 C CNN
F 1 "47346-0001" H 18300 9150 50  0001 L CNN
F 2 "47346-0001" H 18300 9250 50  0001 L CNN
F 3 "http://www.molex.com/pdm_docs/sd/473460001_sd.pdf" H 18300 9150 50  0001 L CNN
F 4 "Micro USB B Receptacle Bottom Mount Assy Molex Right Angle SMT Type B Version 2.0 Micro USB Connector Socket, 30 V ac, 1A 47352 MICRO-USB" H 18300 9050 50  0001 L CNN "Description"
F 5 "538-47346-0001" H 18300 8850 50  0001 L CNN "Mouser Part Number"
F 6 "https://www.mouser.co.uk/ProductDetail/Molex/47346-0001/?qs=c2CV6XM0DweJBWaSeyWeCw%3D%3D" H 18300 8750 50  0001 L CNN "Mouser Price/Stock"
F 7 "Molex" H 18300 8650 50  0001 L CNN "Manufacturer_Name"
F 8 "47346-0001" H 18300 8550 50  0001 L CNN "Manufacturer_Part_Number"
	1    18550 9750
	-1   0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR031
U 1 1 60490EF2
P 18100 9950
F 0 "#PWR031" H 18100 9700 50  0001 C CNN
F 1 "GNDREF" V 18105 9822 50  0000 R CNN
F 2 "" H 18100 9950 50  0001 C CNN
F 3 "" H 18100 9950 50  0001 C CNN
	1    18100 9950
	0    1    -1   0   
$EndComp
Text GLabel 18100 8950 1    50   Output ~ 0
VBUS
NoConn ~ 18100 9850
$Comp
L SamacSys_Parts:ERJ2RKD27R0X R26
U 1 1 604F8B5D
P 17750 9650
F 0 "R26" H 17750 9750 50  0000 C CNN
F 1 "ERJ2RKD27R0X" H 17450 9050 50  0001 L CNN
F 2 "RESC1005X40N" H 17450 9150 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ2RKD27R0X.pdf" H 17450 9050 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ2RKD27R0X - RES, THICK FILM, 27R, 0.5%, 0.063W, 0402" H 17450 8950 50  0001 L CNN "Description"
F 5 "0.4" H 17450 8850 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKD27R0X" H 17450 8750 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKD27R0X" H 17450 8650 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 17450 8550 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ2RKD27R0X" H 17450 8450 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "27" H 17750 9650 50  0000 C CNN "Resistance"
	1    17750 9650
	-1   0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:ERJ2RKD27R0X R27
U 1 1 604FD92A
P 17750 9750
F 0 "R27" H 17750 9650 50  0000 C CNN
F 1 "ERJ2RKD27R0X" H 17450 9150 50  0001 L CNN
F 2 "RESC1005X40N" H 17450 9250 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ2RKD27R0X.pdf" H 17450 9150 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ2RKD27R0X - RES, THICK FILM, 27R, 0.5%, 0.063W, 0402" H 17450 9050 50  0001 L CNN "Description"
F 5 "0.4" H 17450 8950 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKD27R0X" H 17450 8850 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKD27R0X" H 17450 8750 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 17450 8650 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ2RKD27R0X" H 17450 8550 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "27" H 17750 9750 50  0000 C CNN "Resistance"
	1    17750 9750
	-1   0    0    -1  
$EndComp
Text Notes 18800 9450 2    50   ~ 0
Debug
Wire Wire Line
	15900 9650 16250 9650
Wire Wire Line
	15900 9750 16250 9750
Entry Wire Line
	16250 9650 16350 9750
Entry Wire Line
	16250 9750 16350 9850
Wire Bus Line
	16350 9750 16350 9850
Wire Wire Line
	17400 9750 17200 9750
Wire Wire Line
	17200 9650 17400 9650
Entry Wire Line
	17200 9750 17100 9850
Entry Wire Line
	17200 9650 17100 9750
Wire Bus Line
	17100 9750 17100 9850
Text Label 15900 9750 0    50   ~ 0
DEBUG_D-
Text Label 15900 9650 0    50   ~ 0
DEBUG_D+
Text Label 17400 9750 2    50   ~ 0
DEBUG_D+
Text Label 17400 9650 2    50   ~ 0
DEBUG_D-
Entry Wire Line
	17300 7100 17200 7200
Wire Bus Line
	17200 7400 17200 7200
Wire Bus Line
	19000 7400 18500 7400
Text Label 18900 7100 0    50   ~ 0
DATA_D-
Text Label 17300 7100 0    50   ~ 0
DATA_D+
Text Label 19000 8000 2    50   ~ 0
DATA_D+
Text Label 19000 7900 2    50   ~ 0
DATA_D-
Wire Wire Line
	18600 8000 19000 8000
Wire Wire Line
	18600 7900 19000 7900
Wire Wire Line
	17600 7100 17300 7100
Entry Wire Line
	18600 8000 18500 7900
Entry Wire Line
	18600 7900 18500 7800
Entry Wire Line
	18900 7100 19000 7200
Wire Bus Line
	19000 7200 19000 7400
Text Notes 21300 7700 2    50   ~ 0
Data
$Comp
L SamacSys_Parts:ERJ2RKD27R0X R30
U 1 1 604AEC1C
P 19350 8000
F 0 "R30" H 19350 7900 50  0000 C CNN
F 1 "ERJ2RKD27R0X" H 19050 7400 50  0001 L CNN
F 2 "RESC1005X40N" H 19050 7500 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ2RKD27R0X.pdf" H 19050 7400 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ2RKD27R0X - RES, THICK FILM, 27R, 0.5%, 0.063W, 0402" H 19050 7300 50  0001 L CNN "Description"
F 5 "0.4" H 19050 7200 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKD27R0X" H 19050 7100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKD27R0X" H 19050 7000 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 19050 6900 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ2RKD27R0X" H 19050 6800 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "27" H 19350 8000 50  0000 C CNN "Resistance"
	1    19350 8000
	-1   0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:MI1206K601R-10 FB1
U 1 1 604AB922
P 19850 6400
F 0 "FB1" V 19788 6222 50  0000 R CNN
F 1 "MI1206K601R-10" V 19697 6222 50  0000 R CNN
F 2 "BEADC3216X130N" H 19550 6050 50  0001 L CNN
F 3 "http://cdn.lairdtech.com/home/brandworld/files/MI1206K601R-10.pdf" H 19550 5950 50  0001 L CNN
F 4 "LAIRD TECHNOLOGIES - MI1206K601R-10 - FERRITE, BEAD, 3216, 100MHZ, 600R" H 19550 5850 50  0001 L CNN "Description"
F 5 "1.3" H 19550 5750 50  0001 L CNN "Height"
F 6 "N/A" H 19550 5650 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=N%2FA" H 19550 5550 50  0001 L CNN "Mouser Price/Stock"
F 8 "Laird Technologies" H 19550 5450 50  0001 L CNN "Manufacturer_Name"
F 9 "MI1206K601R-10" H 19550 5350 50  0001 L CNN "Manufacturer_Part_Number"
	1    19850 6400
	0    -1   -1   0   
$EndComp
$Comp
L SamacSys_Parts:ERJ2RKD27R0X R29
U 1 1 604A56FC
P 19350 7900
F 0 "R29" H 19350 8000 50  0000 C CNN
F 1 "ERJ2RKD27R0X" H 19050 7300 50  0001 L CNN
F 2 "RESC1005X40N" H 19050 7400 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ2RKD27R0X.pdf" H 19050 7300 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ2RKD27R0X - RES, THICK FILM, 27R, 0.5%, 0.063W, 0402" H 19050 7200 50  0001 L CNN "Description"
F 5 "0.4" H 19050 7100 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKD27R0X" H 19050 7000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKD27R0X" H 19050 6900 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 19050 6800 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ2RKD27R0X" H 19050 6700 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "27" H 19350 7900 50  0000 C CNN "Resistance"
	1    19350 7900
	-1   0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:FT230XS-U IC6
U 1 1 6049F5B2
P 18250 6750
F 0 "IC6" H 18250 7365 50  0000 C CNN
F 1 "FT230XS-U" H 18250 7274 50  0000 C CNN
F 2 "SOP64P599X175-16N" H 17850 5750 50  0001 L CNN
F 3 "https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT230X.pdf" H 17850 5650 50  0001 L CNN
F 4 "USB Interface IC USB to Basic Serial UART IC SSOP-16" H 17850 5550 50  0001 L CNN "Description"
F 5 "1.753" H 17850 5450 50  0001 L CNN "Height"
F 6 "895-FT230XS-U" H 17850 5350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/FTDI/FT230XS-U/?qs=Gp1Yz1mis3UZolxbXux7lw%3D%3D" H 17850 5250 50  0001 L CNN "Mouser Price/Stock"
F 8 "FTDI Chip" H 17850 5150 50  0001 L CNN "Manufacturer_Name"
F 9 "FT230XS-U" H 17850 5050 50  0001 L CNN "Manufacturer_Part_Number"
	1    18250 6750
	1    0    0    -1  
$EndComp
Text GLabel 17600 6600 0    50   Input ~ 0
3v3
NoConn ~ 17600 6500
NoConn ~ 18900 6600
NoConn ~ 18900 6400
NoConn ~ 17600 7000
Text GLabel 18900 6900 2    50   Input ~ 0
3v3
Text GLabel 20350 6800 2    50   Input ~ 0
VBUS
Text GLabel 20000 7800 0    50   Output ~ 0
VBUS
Text GLabel 17600 6400 0    50   Output ~ 0
DATA_RX
Text GLabel 17600 6700 0    50   Input ~ 0
DATA_TX
$Comp
L power:GNDREF #PWR029
U 1 1 60493D03
P 17600 6800
F 0 "#PWR029" H 17600 6550 50  0001 C CNN
F 1 "GNDREF" V 17605 6672 50  0000 R CNN
F 2 "" H 17600 6800 50  0001 C CNN
F 3 "" H 17600 6800 50  0001 C CNN
	1    17600 6800
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR032
U 1 1 60493983
P 18900 6700
F 0 "#PWR032" H 18900 6450 50  0001 C CNN
F 1 "GNDREF" V 18905 6572 50  0000 R CNN
F 2 "" H 18900 6700 50  0001 C CNN
F 3 "" H 18900 6700 50  0001 C CNN
	1    18900 6700
	0    -1   -1   0   
$EndComp
Text Label 16400 9850 0    50   ~ 0
DEBUG_USB_BUS
Wire Bus Line
	16350 9850 17100 9850
Wire Bus Line
	18500 7400 17200 7400
Text Label 17250 7400 0    50   ~ 0
DATA_USB_BUS
$Comp
L SamacSys_Parts:960-23-12-D-AB-0 H1
U 1 1 605D9A93
P 14150 5150
F 0 "H1" H 14150 5323 50  0000 C CNN
F 1 "960-23-12-D-AB-0" H 13900 4550 50  0001 L CNN
F 2 "9602312DAB0" H 13900 4650 50  0001 L CNN
F 3 "http://www.wakefield-vette.com/Portals/0/push pin heat sinks/Push Pin 960 Series Wakefield-Vette Data Sheet.pdf" H 13900 4550 50  0001 L CNN
F 4 "Heat Sinks BGA Heatsink, 23x12mm, Diagonal Plastic Push Pin" H 13900 4450 50  0001 L CNN "Description"
F 5 "12" H 13900 4350 50  0001 L CNN "Height"
F 6 "567-960-23-12-D-AB-0" H 13900 4250 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Wakefield-Vette/960-23-12-D-AB-0?qs=PqoDHHvF64%252BdCFngwMp09w%3D%3D" H 13900 4150 50  0001 L CNN "Mouser Price/Stock"
F 8 "Wakefield-Vette" H 13900 4050 50  0001 L CNN "Manufacturer_Name"
F 9 "960-23-12-D-AB-0" H 13900 3950 50  0001 L CNN "Manufacturer_Part_Number"
	1    14150 5150
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:PH2925U,115 Q1
U 1 1 605F188E
P 11500 5000
F 0 "Q1" H 11500 5365 50  0000 C CNN
F 1 "PH2925U,115" H 11500 5274 50  0000 C CNN
F 2 "PSMN5R660YLX" H 11200 4500 50  0001 L CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/PH2925U.pdf" H 11200 4400 50  0001 L CNN
F 4 "NEXPERIA - PH2925U,115 - MOSFET Transistor, N Channel, 100 A, 25 V, 0.0023 ohm, 4.5 V, 700 mV" H 11200 4300 50  0001 L CNN "Description"
F 5 "" H 11200 4200 50  0001 L CNN "Height"
F 6 "771-PH2925U115" H 11200 4100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Nexperia/PH2925U115?qs=LOCUfHb8d9vnFm%2FhT%252Bjztw%3D%3D" H 11200 4000 50  0001 L CNN "Mouser Price/Stock"
F 8 "Nexperia" H 11200 3900 50  0001 L CNN "Manufacturer_Name"
F 9 "PH2925U,115" H 11200 3800 50  0001 L CNN "Manufacturer_Part_Number"
	1    11500 5000
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:PH2925U,115 Q2
U 1 1 605F2EB4
P 11500 5750
F 0 "Q2" H 11500 6115 50  0000 C CNN
F 1 "PH2925U,115" H 11500 6024 50  0000 C CNN
F 2 "PSMN5R660YLX" H 11200 5250 50  0001 L CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/PH2925U.pdf" H 11200 5150 50  0001 L CNN
F 4 "NEXPERIA - PH2925U,115 - MOSFET Transistor, N Channel, 100 A, 25 V, 0.0023 ohm, 4.5 V, 700 mV" H 11200 5050 50  0001 L CNN "Description"
F 5 "" H 11200 4950 50  0001 L CNN "Height"
F 6 "771-PH2925U115" H 11200 4850 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Nexperia/PH2925U115?qs=LOCUfHb8d9vnFm%2FhT%252Bjztw%3D%3D" H 11200 4750 50  0001 L CNN "Mouser Price/Stock"
F 8 "Nexperia" H 11200 4650 50  0001 L CNN "Manufacturer_Name"
F 9 "PH2925U,115" H 11200 4550 50  0001 L CNN "Manufacturer_Part_Number"
	1    11500 5750
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:PH2925U,115 Q3
U 1 1 605F3196
P 11500 6500
F 0 "Q3" H 11500 6865 50  0000 C CNN
F 1 "PH2925U,115" H 11500 6774 50  0000 C CNN
F 2 "PSMN5R660YLX" H 11200 6000 50  0001 L CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/PH2925U.pdf" H 11200 5900 50  0001 L CNN
F 4 "NEXPERIA - PH2925U,115 - MOSFET Transistor, N Channel, 100 A, 25 V, 0.0023 ohm, 4.5 V, 700 mV" H 11200 5800 50  0001 L CNN "Description"
F 5 "" H 11200 5700 50  0001 L CNN "Height"
F 6 "771-PH2925U115" H 11200 5600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Nexperia/PH2925U115?qs=LOCUfHb8d9vnFm%2FhT%252Bjztw%3D%3D" H 11200 5500 50  0001 L CNN "Mouser Price/Stock"
F 8 "Nexperia" H 11200 5400 50  0001 L CNN "Manufacturer_Name"
F 9 "PH2925U,115" H 11200 5300 50  0001 L CNN "Manufacturer_Part_Number"
	1    11500 6500
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:PH2925U,115 Q4
U 1 1 605F33FF
P 11500 7250
F 0 "Q4" H 11500 7615 50  0000 C CNN
F 1 "PH2925U,115" H 11500 7524 50  0000 C CNN
F 2 "PSMN5R660YLX" H 11200 6750 50  0001 L CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/PH2925U.pdf" H 11200 6650 50  0001 L CNN
F 4 "NEXPERIA - PH2925U,115 - MOSFET Transistor, N Channel, 100 A, 25 V, 0.0023 ohm, 4.5 V, 700 mV" H 11200 6550 50  0001 L CNN "Description"
F 5 "" H 11200 6450 50  0001 L CNN "Height"
F 6 "771-PH2925U115" H 11200 6350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Nexperia/PH2925U115?qs=LOCUfHb8d9vnFm%2FhT%252Bjztw%3D%3D" H 11200 6250 50  0001 L CNN "Mouser Price/Stock"
F 8 "Nexperia" H 11200 6150 50  0001 L CNN "Manufacturer_Name"
F 9 "PH2925U,115" H 11200 6050 50  0001 L CNN "Manufacturer_Part_Number"
	1    11500 7250
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:TLV9101IDBVR IC1
U 1 1 605E3053
P 8500 5250
F 0 "IC1" H 8500 5615 50  0000 C CNN
F 1 "TLV9101IDBVR" H 8500 5524 50  0000 C CNN
F 2 "SOT95P280X145-5N" H 8150 4800 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlv9101.pdf" H 8150 4700 50  0001 L CNN
F 4 "Single 1MHz, 16-V rail-to-rail input/output, low-offset voltage, low-power op amp" H 8150 4600 50  0001 L CNN "Description"
F 5 "1.45" H 8150 4500 50  0001 L CNN "Height"
F 6 "595-TLV9101IDBVR" H 8150 4400 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/TLV9101IDBVR?qs=P1JMDcb91o6ZLUyc%2FHtrGA%3D%3D" H 8150 4300 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 8150 4200 50  0001 L CNN "Manufacturer_Name"
F 9 "TLV9101IDBVR" H 8150 4100 50  0001 L CNN "Manufacturer_Part_Number"
	1    8500 5250
	1    0    0    -1  
$EndComp
Text GLabel 7300 5150 0    50   Output ~ 0
MOS_DRIVE
Text GLabel 11950 4900 2    50   Input ~ 0
MOS_DRIVE
Text GLabel 11950 5650 2    50   Input ~ 0
MOS_DRIVE
Text GLabel 11950 6400 2    50   Input ~ 0
MOS_DRIVE
Text GLabel 11950 7150 2    50   Input ~ 0
MOS_DRIVE
$Comp
L SamacSys_Parts:CRGP2512F12R R6
U 1 1 606074D4
P 10150 4900
F 0 "R6" H 10100 4900 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4300 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4400 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 4850 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4200 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4100 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 3900 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 3800 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 3700 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 4900 50  0000 C CNN "Resistance"
	1    10150 4900
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R7
U 1 1 6060AC31
P 10150 5000
F 0 "R7" H 10100 5000 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4400 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4500 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 4950 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4300 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4200 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4000 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 3900 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 3800 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5000 50  0000 C CNN "Resistance"
	1    10150 5000
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R8
U 1 1 6060AD10
P 10150 5100
F 0 "R8" H 10100 5100 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4500 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4600 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5050 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4400 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4300 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4200 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4100 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4000 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 3900 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5100 50  0000 C CNN "Resistance"
	1    10150 5100
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R9
U 1 1 6060ADE3
P 10150 5200
F 0 "R9" H 10100 5200 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4600 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4700 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5150 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4500 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4400 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4300 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4200 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4100 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4000 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5200 50  0000 C CNN "Resistance"
	1    10150 5200
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R10
U 1 1 6060B5E2
P 10150 5300
F 0 "R10" H 10100 5300 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4700 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4800 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5250 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4600 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4500 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4400 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4300 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4200 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4100 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5300 50  0000 C CNN "Resistance"
	1    10150 5300
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R11
U 1 1 6060B6C1
P 10150 5400
F 0 "R11" H 10100 5400 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4800 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 4900 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5350 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4700 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4600 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4400 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4300 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4200 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5400 50  0000 C CNN "Resistance"
	1    10150 5400
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R12
U 1 1 6060B794
P 10150 5500
F 0 "R12" H 10100 5500 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 4900 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5000 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5450 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4800 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4700 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4500 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4400 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4300 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5500 50  0000 C CNN "Resistance"
	1    10150 5500
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R13
U 1 1 6060BA1F
P 10150 5600
F 0 "R13" H 10100 5600 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 5000 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5100 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5550 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 4900 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4800 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4700 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4600 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4500 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4400 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5600 50  0000 C CNN "Resistance"
	1    10150 5600
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R14
U 1 1 6060BAF2
P 10150 5700
F 0 "R14" H 10100 5700 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 5100 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5200 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5650 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 5000 50  0001 L CNN "Description"
F 5 "0.65" H 9900 4900 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4800 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4700 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4600 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4500 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5700 50  0000 C CNN "Resistance"
	1    10150 5700
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R15
U 1 1 6060BBC5
P 10150 5800
F 0 "R15" H 10100 5800 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 5200 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5300 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5750 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 5100 50  0001 L CNN "Description"
F 5 "0.65" H 9900 5000 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 4900 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4800 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4700 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4600 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5800 50  0000 C CNN "Resistance"
	1    10150 5800
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R16
U 1 1 6060BE60
P 10150 5900
F 0 "R16" H 10100 5900 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 5300 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5400 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5850 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 5200 50  0001 L CNN "Description"
F 5 "0.65" H 9900 5100 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 5000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 4900 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4800 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4700 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 5900 50  0000 C CNN "Resistance"
	1    10150 5900
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:CRGP2512F12R R17
U 1 1 6060BF33
P 10150 6000
F 0 "R17" H 10100 6000 50  0000 C CNN
F 1 "CRGP2512F12R" H 9900 5400 50  0001 L CNN
F 2 "RESC6432X65N" H 9900 5500 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7F9-1773463-9%7FA%7Fpdf%7FEnglish%7FENG_DS_9-1773463-9_A.pdf%7F2176331-2" H 10700 5950 50  0001 L CNN
F 4 "Resistor Thick Film Pulse 2512 12R 1%" H 9900 5300 50  0001 L CNN "Description"
F 5 "0.65" H 9900 5200 50  0001 L CNN "Height"
F 6 "279-CRGP2512F12R" H 9900 5100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/TE-Connectivity-Holsworthy/CRGP2512F12R/?qs=wUXugUrL1qxcIPHB4vXKuA%3D%3D" H 9900 5000 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 9900 4900 50  0001 L CNN "Manufacturer_Name"
F 9 "CRGP2512F12R" H 9900 4800 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "12" H 10250 6000 50  0000 C CNN "Resistance"
	1    10150 6000
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:960-23-12-D-AB-0 H2
U 1 1 6060F9CD
P 14150 5650
F 0 "H2" H 14150 5823 50  0000 C CNN
F 1 "960-23-12-D-AB-0" H 13900 5050 50  0001 L CNN
F 2 "9602312DAB0" H 13900 5150 50  0001 L CNN
F 3 "http://www.wakefield-vette.com/Portals/0/push pin heat sinks/Push Pin 960 Series Wakefield-Vette Data Sheet.pdf" H 13900 5050 50  0001 L CNN
F 4 "Heat Sinks BGA Heatsink, 23x12mm, Diagonal Plastic Push Pin" H 13900 4950 50  0001 L CNN "Description"
F 5 "12" H 13900 4850 50  0001 L CNN "Height"
F 6 "567-960-23-12-D-AB-0" H 13900 4750 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Wakefield-Vette/960-23-12-D-AB-0?qs=PqoDHHvF64%252BdCFngwMp09w%3D%3D" H 13900 4650 50  0001 L CNN "Mouser Price/Stock"
F 8 "Wakefield-Vette" H 13900 4550 50  0001 L CNN "Manufacturer_Name"
F 9 "960-23-12-D-AB-0" H 13900 4450 50  0001 L CNN "Manufacturer_Part_Number"
	1    14150 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 4900 11050 5000
Connection ~ 11050 5750
Wire Wire Line
	11050 5750 11050 5850
Connection ~ 11050 5850
Wire Wire Line
	11050 5850 11050 6400
Connection ~ 11050 6400
Wire Wire Line
	11050 6400 11050 6500
Connection ~ 11050 6500
Wire Wire Line
	11050 6500 11050 6600
Connection ~ 11050 6600
Wire Wire Line
	11050 6600 11050 7150
Connection ~ 11050 7150
Wire Wire Line
	11050 7150 11050 7250
Connection ~ 11050 7250
Wire Wire Line
	11050 7250 11050 7350
Wire Wire Line
	11050 4900 10500 4900
Wire Wire Line
	10500 4900 10500 5000
Connection ~ 10500 4900
Connection ~ 10500 5000
Wire Wire Line
	10500 5000 10500 5100
Connection ~ 10500 5100
Wire Wire Line
	10500 5100 10500 5200
Connection ~ 10500 5200
Wire Wire Line
	10500 5200 10500 5300
Connection ~ 10500 5300
Wire Wire Line
	10500 5300 10500 5400
Connection ~ 10500 5400
Wire Wire Line
	10500 5400 10500 5500
Connection ~ 10500 5500
Wire Wire Line
	10500 5500 10500 5600
Connection ~ 10500 5600
Wire Wire Line
	10500 5600 10500 5700
Connection ~ 10500 5700
Wire Wire Line
	10500 5700 10500 5800
Connection ~ 10500 5800
Wire Wire Line
	10500 5800 10500 5900
Connection ~ 10500 5900
Wire Wire Line
	10500 5900 10500 6000
Text Label 11050 4900 2    50   ~ 0
MOS_SOURCE
Wire Wire Line
	9800 4900 9800 5000
Connection ~ 9800 5000
Wire Wire Line
	9800 5000 9800 5100
Connection ~ 9800 5100
Wire Wire Line
	9800 5100 9800 5200
Connection ~ 9800 5200
Wire Wire Line
	9800 5200 9800 5300
Connection ~ 9800 5300
Wire Wire Line
	9800 5300 9800 5400
Connection ~ 9800 5400
Wire Wire Line
	9800 5400 9800 5500
Connection ~ 9800 5500
Wire Wire Line
	9800 5500 9800 5600
Connection ~ 9800 5600
Wire Wire Line
	9800 5600 9800 5700
Connection ~ 9800 5700
Wire Wire Line
	9800 5700 9800 5800
Connection ~ 9800 5800
Wire Wire Line
	9800 5800 9800 5900
Connection ~ 9800 5900
Wire Wire Line
	9800 5900 9800 6000
$Comp
L power:GNDREF #PWR04
U 1 1 606522E0
P 9800 6000
F 0 "#PWR04" H 9800 5750 50  0001 C CNN
F 1 "GNDREF" H 9805 5827 50  0000 C CNN
F 2 "" H 9800 6000 50  0001 C CNN
F 3 "" H 9800 6000 50  0001 C CNN
	1    9800 6000
	1    0    0    -1  
$EndComp
Connection ~ 9800 6000
Text GLabel 11950 5750 2    50   Input ~ 0
BATT
Text GLabel 11950 5000 2    50   Input ~ 0
BATT
Text GLabel 11950 6500 2    50   Input ~ 0
BATT
Text GLabel 11950 7250 2    50   Input ~ 0
BATT
Connection ~ 11050 4900
Connection ~ 11050 5000
Wire Wire Line
	11050 5000 11050 5100
Connection ~ 11050 5100
Wire Wire Line
	11050 5100 11050 5650
Connection ~ 11050 5650
Wire Wire Line
	11050 5650 11050 5750
Text GLabel 9000 5250 2    50   Input ~ 0
VBUS
$Comp
L power:GNDREF #PWR02
U 1 1 6066B1E6
P 8000 5250
F 0 "#PWR02" H 8000 5000 50  0001 C CNN
F 1 "GNDREF" V 8005 5122 50  0000 R CNN
F 2 "" H 8000 5250 50  0001 C CNN
F 3 "" H 8000 5250 50  0001 C CNN
	1    8000 5250
	0    1    -1   0   
$EndComp
Text GLabel 9000 5150 2    50   Input ~ 0
MOS_SOURCE
Text GLabel 14100 9650 0    50   Output ~ 0
MOS_SIGNAL
$Comp
L SamacSys_Parts:ERJ-S02F1002X R1
U 1 1 606D72DC
P 7400 5700
F 0 "R1" H 7400 5925 50  0000 C CNN
F 1 "ERJ-S02F1002X" H 7100 5100 50  0001 L CNN
F 2 "RESC1005X40N" H 7100 5200 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDP0000/AOA0000C334.pdf" H 7950 5650 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 0402 10Kohms 1% Anti-Sulfur" H 7100 5000 50  0001 L CNN "Description"
F 5 "0.4" H 7100 4900 50  0001 L CNN "Height"
F 6 "667-ERJ-S02F1002X" H 7100 4800 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Panasonic/ERJ-S02F1002X?qs=Zyl8A9hlmJre8ZnVBnn2RQ%3D%3D" H 7100 4700 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 7100 4600 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-S02F1002X" H 7100 4500 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 k" H 7400 5834 50  0000 C CNN "Resistance"
	1    7400 5700
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:RC0402FR-075KL R3
U 1 1 606D852E
P 7750 5700
F 0 "R3" V 7704 5788 50  0000 L CNN
F 1 "RC0402FR-075KL" H 7450 5100 50  0001 L CNN
F 2 "RESC1005X40N" H 7450 5200 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC0402_51_RoHS_L_6_r.pdf" H 7450 5100 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 5K ohm 1% 50V General Purpose" H 7450 5000 50  0001 L CNN "Description"
F 5 "0.4" H 7450 4900 50  0001 L CNN "Height"
F 6 "603-RC0402FR-075KL" H 7450 4800 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Yageo/RC0402FR-075KL/?qs=9h0bZHM%2F3zL5adZvfm%252BR4Q%3D%3D" H 7450 4700 50  0001 L CNN "Mouser Price/Stock"
F 8 "YAGEO (PHYCOMP)" H 7450 4600 50  0001 L CNN "Manufacturer_Name"
F 9 "RC0402FR-075KL" H 7450 4500 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "5 k" V 7795 5788 50  0000 L CNN "Resistance"
	1    7750 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 5350 7750 5350
Connection ~ 7750 5350
$Comp
L power:GNDREF #PWR01
U 1 1 606DE522
P 7750 6050
F 0 "#PWR01" H 7750 5800 50  0001 C CNN
F 1 "GNDREF" H 7755 5877 50  0000 C CNN
F 2 "" H 7750 6050 50  0001 C CNN
F 3 "" H 7750 6050 50  0001 C CNN
	1    7750 6050
	1    0    0    -1  
$EndComp
Text GLabel 7400 6050 3    50   Input ~ 0
MOS_SIGNAL
NoConn ~ 15000 11350
NoConn ~ 15200 11350
NoConn ~ 15100 11350
NoConn ~ 18900 7000
$Comp
L SamacSys_Parts:SPX3819M5-L-3-3_TR IC7
U 1 1 6079BAA8
P 18500 11000
F 0 "IC7" H 18500 11365 50  0000 C CNN
F 1 "SPX3819M5-L-3-3_TR" H 18500 11274 50  0000 C CNN
F 2 "SOT95P280X145-5N" H 18100 10500 50  0001 L CNN
F 3 "https://www.exar.com/ds/spx3819.pdf" H 18100 10400 50  0001 L CNN
F 4 "LDO Voltage Regulators 500mA LOW NOISE LDO" H 18100 10300 50  0001 L CNN "Description"
F 5 "1.45" H 18100 10200 50  0001 L CNN "Height"
F 6 "701-SPX3819M5-L-33TR" H 18100 10100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/MaxLinear/SPX3819M5-L-3-3-TR/?qs=S%2FCBhQS5rCp1Bb7a%252BD%2FCBQ%3D%3D" H 18100 10000 50  0001 L CNN "Mouser Price/Stock"
F 8 "MaxLinear, Inc." H 18100 9900 50  0001 L CNN "Manufacturer_Name"
F 9 "SPX3819M5-L-3-3/TR" H 18100 9800 50  0001 L CNN "Manufacturer_Part_Number"
	1    18500 11000
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:0603ZD106KAT4A C11
U 1 1 607AE209
P 19100 11250
F 0 "C11" V 19054 11378 50  0000 L CNN
F 1 "0603ZD106KAT4A" H 18800 10650 50  0001 L CNN
F 2 "CAPC1608X90N" H 18800 10750 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06031A100FAT2A.pdf" H 18800 10650 50  0001 L CNN
F 4 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10V 10uF 0603 10% XR5" H 18800 10550 50  0001 L CNN "Description"
F 5 "0.9" H 18800 10450 50  0001 L CNN "Height"
F 6 "581-0603ZD106KAT4A" H 18800 10350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/0603ZD106KAT4A?qs=%252BdQmOuGyFcGtso703HlSDw%3D%3D" H 18800 10250 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 18800 10150 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZD106KAT4A" H 18800 10050 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 uF" V 19145 11378 50  0000 L CNN "Capacitance"
	1    19100 11250
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:0603ZD106KAT4A C10
U 1 1 607AF243
P 17100 11350
F 0 "C10" V 17054 11478 50  0000 L CNN
F 1 "0603ZD106KAT4A" H 16800 10750 50  0001 L CNN
F 2 "CAPC1608X90N" H 16800 10850 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06031A100FAT2A.pdf" H 16800 10750 50  0001 L CNN
F 4 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10V 10uF 0603 10% XR5" H 16800 10650 50  0001 L CNN "Description"
F 5 "0.9" H 16800 10550 50  0001 L CNN "Height"
F 6 "581-0603ZD106KAT4A" H 16800 10450 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/0603ZD106KAT4A?qs=%252BdQmOuGyFcGtso703HlSDw%3D%3D" H 16800 10350 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 16800 10250 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZD106KAT4A" H 16800 10150 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 uF" V 17145 11478 50  0000 L CNN "Capacitance"
	1    17100 11350
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:ERJ-U02F1003X R25
U 1 1 607B30B4
P 17450 11100
F 0 "R25" H 17450 11200 50  0000 C CNN
F 1 "ERJ-U02F1003X" H 17150 10500 50  0001 L CNN
F 2 "RESC1005X40N" H 17150 10600 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ-U02F1003X.pdf" H 17150 10500 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ-U02F1003X - RES, THICK FILM, 100K, 1%, 0.1W, 0402" H 17150 10400 50  0001 L CNN "Description"
F 5 "0.4" H 17150 10300 50  0001 L CNN "Height"
F 6 "667-ERJ-U02F1003X" H 17150 10200 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-U02F1003X" H 17150 10100 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 17150 10000 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-U02F1003X" H 17150 9900 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 k" H 17450 11100 50  0000 C CNN "Resistance"
	1    17450 11100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR027
U 1 1 607B4DE3
P 17100 11600
F 0 "#PWR027" H 17100 11350 50  0001 C CNN
F 1 "GNDREF" H 17105 11427 50  0000 C CNN
F 2 "" H 17100 11600 50  0001 C CNN
F 3 "" H 17100 11600 50  0001 C CNN
	1    17100 11600
	1    0    0    -1  
$EndComp
Wire Wire Line
	17900 11100 17800 11100
$Comp
L power:GNDREF #PWR030
U 1 1 607B70D5
P 17850 11300
F 0 "#PWR030" H 17850 11050 50  0001 C CNN
F 1 "GNDREF" H 17855 11127 50  0000 C CNN
F 2 "" H 17850 11300 50  0001 C CNN
F 3 "" H 17850 11300 50  0001 C CNN
	1    17850 11300
	1    0    0    -1  
$EndComp
Wire Wire Line
	17850 11300 17850 11000
Wire Wire Line
	17850 11000 17900 11000
Wire Wire Line
	17900 10900 17100 10900
Wire Wire Line
	17100 10900 17100 11100
Connection ~ 17100 11100
$Comp
L power:GNDREF #PWR035
U 1 1 607C8D65
P 19100 11500
F 0 "#PWR035" H 19100 11250 50  0001 C CNN
F 1 "GNDREF" H 19105 11327 50  0000 C CNN
F 2 "" H 19100 11500 50  0001 C CNN
F 3 "" H 19100 11500 50  0001 C CNN
	1    19100 11500
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:0603ZC105KAT2A C15
U 1 1 607C9C2F
P 19550 11250
F 0 "C15" V 19504 11378 50  0000 L CNN
F 1 "0603ZC105KAT2A" H 19250 10550 50  0001 L CNN
F 2 "CAPC1608X90N" H 19250 10650 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/0603ZC105KAT2A.pdf" H 19250 10550 50  0001 L CNN
F 4 "AVX - 0603ZC105KAT2A - CAP, MLCC, X7R, 1UF, 10V, 0603" H 19250 10450 50  0001 L CNN "Description"
F 5 "0.9" H 19250 10350 50  0001 L CNN "Height"
F 6 "581-0603ZC105KAT2A" H 19250 10250 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=581-0603ZC105KAT2A" H 19250 10150 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 19250 10050 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZC105KAT2A" H 19250 9950 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "1 uF" V 19595 11378 50  0000 L CNN "Capacitance"
	1    19550 11250
	0    1    1    0   
$EndComp
Wire Wire Line
	19550 11000 19100 11000
Connection ~ 19100 11000
Text GLabel 19550 11000 2    50   Output ~ 0
3v3
NoConn ~ 19100 10900
$Comp
L power:GNDREF #PWR036
U 1 1 607CE95D
P 19550 11500
F 0 "#PWR036" H 19550 11250 50  0001 C CNN
F 1 "GNDREF" H 19555 11327 50  0000 C CNN
F 2 "" H 19550 11500 50  0001 C CNN
F 3 "" H 19550 11500 50  0001 C CNN
	1    19550 11500
	1    0    0    -1  
$EndComp
Text GLabel 17100 10900 1    50   Input ~ 0
VBUS
Text GLabel 11950 5100 2    50   Input ~ 0
BATT
Text GLabel 11950 5850 2    50   Input ~ 0
BATT
Text GLabel 11950 6600 2    50   Input ~ 0
BATT
Text GLabel 11950 7350 2    50   Input ~ 0
BATT
NoConn ~ 14550 5150
NoConn ~ 14550 5650
NoConn ~ 13750 5650
NoConn ~ 13750 5150
Text GLabel 13050 10250 0    50   Input ~ 0
MOS_SOURCE
$Comp
L Connector:TestPoint_Probe TP3
U 1 1 608A95B3
P 14100 10250
F 0 "TP3" H 14165 10480 50  0001 C CNN
F 1 "TestPoint_Probe" H 14165 10410 50  0001 C CNN
F 2 "Measurement_Points:Standard_Probe_Point" H 14100 9950 50  0001 C CNN
F 3 "~" H 14300 10250 50  0001 C CNN
	1    14100 10250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP5
U 1 1 608AD09E
P 19100 11000
F 0 "TP5" H 19165 11230 50  0001 C CNN
F 1 "TestPoint_Probe" H 19165 11160 50  0001 C CNN
F 2 "Measurement_Points:Standard_Probe_Point" H 19100 10700 50  0001 C CNN
F 3 "~" H 19300 11000 50  0001 C CNN
	1    19100 11000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP1
U 1 1 608AD660
P 7750 5350
F 0 "TP1" H 7815 5580 50  0001 C CNN
F 1 "TestPoint_Probe" H 7815 5510 50  0001 C CNN
F 2 "Measurement_Points:Standard_Probe_Point" H 7750 5050 50  0001 C CNN
F 3 "~" H 7950 5350 50  0001 C CNN
	1    7750 5350
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint_Probe TP2
U 1 1 608B6B7C
P 8000 5150
F 0 "TP2" H 8065 5380 50  0001 C CNN
F 1 "TestPoint_Probe" H 8065 5310 50  0001 C CNN
F 2 "Measurement_Points:Standard_Probe_Point" H 8000 4850 50  0001 C CNN
F 3 "~" H 8200 5150 50  0001 C CNN
	1    8000 5150
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:ERJ-S02F1002X R19
U 1 1 608B9A45
P 12350 10500
F 0 "R19" V 12250 10600 50  0000 C CNN
F 1 "ERJ-S02F1002X" H 12050 9900 50  0001 L CNN
F 2 "RESC1005X40N" H 12050 10000 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDP0000/AOA0000C334.pdf" H 12900 10450 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 0402 10Kohms 1% Anti-Sulfur" H 12050 9800 50  0001 L CNN "Description"
F 5 "0.4" H 12050 9700 50  0001 L CNN "Height"
F 6 "667-ERJ-S02F1002X" H 12050 9600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Panasonic/ERJ-S02F1002X?qs=Zyl8A9hlmJre8ZnVBnn2RQ%3D%3D" H 12050 9500 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 12050 9400 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-S02F1002X" H 12050 9300 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 k" H 12350 10500 50  0000 C CNN "Resistance"
	1    12350 10500
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:RC0402FR-075KL R18
U 1 1 608BBC96
P 12000 10150
F 0 "R18" H 11850 10250 50  0000 L CNN
F 1 "RC0402FR-075KL" H 11700 9550 50  0001 L CNN
F 2 "RESC1005X40N" H 11700 9650 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC0402_51_RoHS_L_6_r.pdf" H 11700 9550 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 5K ohm 1% 50V General Purpose" H 11700 9450 50  0001 L CNN "Description"
F 5 "0.4" H 11700 9350 50  0001 L CNN "Height"
F 6 "603-RC0402FR-075KL" H 11700 9250 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Yageo/RC0402FR-075KL/?qs=9h0bZHM%2F3zL5adZvfm%252BR4Q%3D%3D" H 11700 9150 50  0001 L CNN "Mouser Price/Stock"
F 8 "YAGEO (PHYCOMP)" H 11700 9050 50  0001 L CNN "Manufacturer_Name"
F 9 "RC0402FR-075KL" H 11700 8950 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "5 k" H 11950 10150 50  0000 L CNN "Resistance"
	1    12000 10150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR07
U 1 1 608C1A6C
P 12350 10850
F 0 "#PWR07" H 12350 10600 50  0001 C CNN
F 1 "GNDREF" H 12355 10677 50  0000 C CNN
F 2 "" H 12350 10850 50  0001 C CNN
F 3 "" H 12350 10850 50  0001 C CNN
	1    12350 10850
	1    0    0    -1  
$EndComp
Wire Wire Line
	12350 10150 14100 10150
Connection ~ 12350 10150
$Comp
L SamacSys_Parts:RC0402FR-075KL R20
U 1 1 608CC402
P 13400 10250
F 0 "R20" H 13250 10150 50  0000 L CNN
F 1 "RC0402FR-075KL" H 13100 9650 50  0001 L CNN
F 2 "RESC1005X40N" H 13100 9750 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC0402_51_RoHS_L_6_r.pdf" H 13100 9650 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 5K ohm 1% 50V General Purpose" H 13100 9550 50  0001 L CNN "Description"
F 5 "0.4" H 13100 9450 50  0001 L CNN "Height"
F 6 "603-RC0402FR-075KL" H 13100 9350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Yageo/RC0402FR-075KL/?qs=9h0bZHM%2F3zL5adZvfm%252BR4Q%3D%3D" H 13100 9250 50  0001 L CNN "Mouser Price/Stock"
F 8 "YAGEO (PHYCOMP)" H 13100 9150 50  0001 L CNN "Manufacturer_Name"
F 9 "RC0402FR-075KL" H 13100 9050 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "5 k" H 13350 10250 50  0000 L CNN "Resistance"
	1    13400 10250
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 10250 14100 10250
Connection ~ 14100 10250
$Comp
L SamacSys_Parts:ERJ-S02F1002X R21
U 1 1 608D4B46
P 13750 10600
F 0 "R21" V 13650 10700 50  0000 C CNN
F 1 "ERJ-S02F1002X" H 13450 10000 50  0001 L CNN
F 2 "RESC1005X40N" H 13450 10100 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDP0000/AOA0000C334.pdf" H 14300 10550 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 0402 10Kohms 1% Anti-Sulfur" H 13450 9900 50  0001 L CNN "Description"
F 5 "0.4" H 13450 9800 50  0001 L CNN "Height"
F 6 "667-ERJ-S02F1002X" H 13450 9700 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Panasonic/ERJ-S02F1002X?qs=Zyl8A9hlmJre8ZnVBnn2RQ%3D%3D" H 13450 9600 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 13450 9500 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-S02F1002X" H 13450 9400 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 k" H 13750 10600 50  0000 C CNN "Resistance"
	1    13750 10600
	0    1    1    0   
$EndComp
Connection ~ 13750 10250
$Comp
L power:GNDREF #PWR012
U 1 1 608D5730
P 13750 10950
F 0 "#PWR012" H 13750 10700 50  0001 C CNN
F 1 "GNDREF" H 13755 10777 50  0000 C CNN
F 2 "" H 13750 10950 50  0001 C CNN
F 3 "" H 13750 10950 50  0001 C CNN
	1    13750 10950
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:LM4040D25IDCKR IC5
U 1 1 60745293
P 16250 12150
F 0 "IC5" H 16250 12515 50  0000 C CNN
F 1 "LM4040D25IDCKR" H 16250 12424 50  0000 C CNN
F 2 "SOT65P210X110-5N" H 15850 11650 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/lm4040" H 15850 11550 50  0001 L CNN
F 4 "Precision Micropower Shunt Voltage Reference" H 15850 11450 50  0001 L CNN "Description"
F 5 "1.1" H 15850 11350 50  0001 L CNN "Height"
F 6 "595-LM4040D25IDCKR" H 15850 11250 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/LM4040D25IDCKR?qs=paYhMW8qfivGgJ8gGFiCWw%3D%3D" H 15850 11150 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 15850 11050 50  0001 L CNN "Manufacturer_Name"
F 9 "LM4040D25IDCKR" H 15850 10950 50  0001 L CNN "Manufacturer_Part_Number"
	1    16250 12150
	1    0    0    -1  
$EndComp
NoConn ~ 16900 12050
NoConn ~ 16900 12150
NoConn ~ 15600 12150
$Comp
L SamacSys_Parts:ERJ-U02F1001X R22
U 1 1 6074ECDD
P 15600 12600
F 0 "R22" V 15646 12522 50  0000 R CNN
F 1 "ERJ-U02F1001X" H 15300 12000 50  0001 L CNN
F 2 "RESC1005X40N" H 15300 11900 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ-U02F1001X.pdf" H 15300 11800 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 0402 1% 1.0Kohm Anti-Sulfur AEC-Q200" H 15300 11700 50  0001 L CNN "Description"
F 5 "0.4" H 15300 11600 50  0001 L CNN "Height"
F 6 "667-ERJ-U02F1001X" H 15300 11500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-U02F1001X" H 15300 11400 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 15300 11300 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-U02F1001X" H 15300 11200 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "1 k" V 15555 12522 50  0000 R CNN "Resistance"
	1    15600 12600
	0    -1   -1   0   
$EndComp
Text GLabel 15600 12950 3    50   Input ~ 0
3v3
Text GLabel 15600 12250 0    50   Output ~ 0
VREF
$Comp
L power:GNDREF #PWR022
U 1 1 60752BE2
P 15600 12050
F 0 "#PWR022" H 15600 11800 50  0001 C CNN
F 1 "GNDREF" V 15605 11922 50  0000 R CNN
F 2 "" H 15600 12050 50  0001 C CNN
F 3 "" H 15600 12050 50  0001 C CNN
	1    15600 12050
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:ERJ2RKD27R0X R2
U 1 1 607541C1
P 7650 5150
F 0 "R2" H 7650 5375 50  0000 C CNN
F 1 "ERJ2RKD27R0X" H 7350 4550 50  0001 L CNN
F 2 "RESC1005X40N" H 7350 4650 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ2RKD27R0X.pdf" H 7350 4550 50  0001 L CNN
F 4 "PANASONIC ELECTRONIC COMPONENTS - ERJ2RKD27R0X - RES, THICK FILM, 27R, 0.5%, 0.063W, 0402" H 7350 4450 50  0001 L CNN "Description"
F 5 "0.4" H 7350 4350 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKD27R0X" H 7350 4250 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKD27R0X" H 7350 4150 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 7350 4050 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ2RKD27R0X" H 7350 3950 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "27" H 7650 5284 50  0000 C CNN "Resistance"
	1    7650 5150
	1    0    0    -1  
$EndComp
Connection ~ 8000 5150
Wire Wire Line
	7750 5350 7400 5350
$Comp
L Connector:TestPoint_Probe TP4
U 1 1 60758BC8
P 14150 6100
F 0 "TP4" H 14215 6330 50  0001 C CNN
F 1 "TestPoint_Probe" H 14215 6260 50  0001 C CNN
F 2 "Measurement_Points:Standard_Probe_Point" H 14150 5800 50  0001 C CNN
F 3 "~" H 14350 6100 50  0001 C CNN
	1    14150 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR015
U 1 1 607595AE
P 14150 6100
F 0 "#PWR015" H 14150 5850 50  0001 C CNN
F 1 "GNDREF" H 14155 5927 50  0000 C CNN
F 2 "" H 14150 6100 50  0001 C CNN
F 3 "" H 14150 6100 50  0001 C CNN
	1    14150 6100
	1    0    0    -1  
$EndComp
Text GLabel 14600 8650 1    50   Input ~ 0
VREF
$Comp
L SamacSys_Parts:0603ZC103KAT2A C17
U 1 1 60810129
P 20350 7050
F 0 "C17" V 20304 7178 50  0000 L CNN
F 1 "0603ZC103KAT2A" H 20050 6550 50  0001 L CNN
F 2 "CAPC1608X90N" H 20050 6450 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/0603ZC103KAT2A.pdf" H 20050 6350 50  0001 L CNN
F 4 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10V .01uF X7R 0603 10%" H 20050 6250 50  0001 L CNN "Description"
F 5 "0.9" H 20050 6150 50  0001 L CNN "Height"
F 6 "581-0603ZC103K" H 20050 6050 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=581-0603ZC103K" H 20050 5950 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 20050 5850 50  0001 L CNN "Manufacturer_Name"
F 9 "0603ZC103KAT2A" H 20050 5750 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 nF" V 20395 7178 50  0000 L CNN "Capacitance"
	1    20350 7050
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR037
U 1 1 608129C1
P 20350 7300
F 0 "#PWR037" H 20350 7050 50  0001 C CNN
F 1 "GNDREF" H 20355 7127 50  0000 C CNN
F 2 "" H 20350 7300 50  0001 C CNN
F 3 "" H 20350 7300 50  0001 C CNN
	1    20350 7300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR038
U 1 1 604911F4
P 20600 8200
F 0 "#PWR038" H 20600 7950 50  0001 C CNN
F 1 "GNDREF" V 20605 8072 50  0000 R CNN
F 2 "" H 20600 8200 50  0001 C CNN
F 3 "" H 20600 8200 50  0001 C CNN
	1    20600 8200
	0    1    -1   0   
$EndComp
NoConn ~ 20600 8100
$Comp
L SamacSys_Parts:TAJA475K010RNJV C16
U 1 1 6082C94F
P 19800 7050
F 0 "C16" V 19750 7250 50  0000 C CNN
F 1 "TAJA475K010RNJV" H 19450 6550 50  0001 L CNN
F 2 "CAPPM3216X180N" H 19450 6450 50  0001 L CNN
F 3 "" H 19450 6350 50  0001 L CNN
F 4 "Tantalum Capacitors - Solid SMD 10V 4.7uF 10% 1206 ESR = 5 Ohms" H 19450 6250 50  0001 L CNN "Description"
F 5 "1.8" H 19450 6150 50  0001 L CNN "Height"
F 6 "581-TAJA475K010RNJV" H 19450 6050 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/TAJA475K010RNJV?qs=VymPLiRQZITld0CPYYWYtw%3D%3D" H 19450 5950 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 19450 5850 50  0001 L CNN "Manufacturer_Name"
F 9 "TAJA475K010RNJV" H 19450 5750 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "4.7 uF" V 19850 7300 50  0000 C CNN "Capacitance"
	1    19800 7050
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:06033C104KAT2A C12
U 1 1 6082D5C0
P 19350 7050
F 0 "C12" V 19300 7250 50  0000 C CNN
F 1 "06033C104KAT2A" H 19050 6450 50  0001 L CNN
F 2 "CAPC1608X90N" H 19050 6550 50  0001 L CNN
F 3 "" H 19050 6450 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 19050 6350 50  0001 L CNN "Description"
F 5 "0.9" H 19050 6250 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 19050 6150 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 19050 6050 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 19050 5950 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 19050 5850 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" V 19400 7250 50  0000 C CNN "Capacitance"
	1    19350 7050
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:06033C104KAT2A C8
U 1 1 60838477
P 15900 6400
F 0 "C8" H 15900 6665 50  0000 C CNN
F 1 "06033C104KAT2A" H 15600 5800 50  0001 L CNN
F 2 "CAPC1608X90N" H 15600 5900 50  0001 L CNN
F 3 "" H 15600 5800 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 15600 5700 50  0001 L CNN "Description"
F 5 "0.9" H 15600 5600 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 15600 5500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 15600 5400 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 15600 5300 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 15600 5200 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" H 15900 6574 50  0000 C CNN "Capacitance"
	1    15900 6400
	0    1    1    0   
$EndComp
Text GLabel 15900 6150 0    50   Input ~ 0
3v3
$Comp
L power:GNDREF #PWR023
U 1 1 60839A37
P 15900 6650
F 0 "#PWR023" H 15900 6400 50  0001 C CNN
F 1 "GNDREF" H 15905 6477 50  0000 C CNN
F 2 "" H 15900 6650 50  0001 C CNN
F 3 "" H 15900 6650 50  0001 C CNN
	1    15900 6650
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:M50-3600542 J1
U 1 1 6085A349
P 14000 7750
F 0 "J1" H 14000 8215 50  0000 C CNN
F 1 "M50-3600542" H 14000 8124 50  0000 C CNN
F 2 "M503600542" H 13750 7250 50  0001 L CNN
F 3 "https://www.mouser.it/datasheet/2/181/M50_360-1133637.pdf" H 13750 7150 50  0001 L CNN
F 4 "Headers & Wire Housings 5+5 DIL PIN HDR SMT Au/Sn" H 13750 7050 50  0001 L CNN "Description"
F 5 "5.13" H 13750 6950 50  0001 L CNN "Height"
F 6 "855-M50-3600542" H 13750 6850 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Harwin/M50-3600542/?qs=9fQaSFfsqsz29hu1Jz0tTA%3D%3D" H 13750 6750 50  0001 L CNN "Mouser Price/Stock"
F 8 "Harwin" H 13750 6650 50  0001 L CNN "Manufacturer_Name"
F 9 "M50-3600542" H 13750 6550 50  0001 L CNN "Manufacturer_Part_Number"
	1    14000 7750
	1    0    0    -1  
$EndComp
Text GLabel 13550 7550 0    50   Input ~ 0
3v3
$Comp
L power:GNDREF #PWR011
U 1 1 6085CC8A
P 13550 7650
F 0 "#PWR011" H 13550 7400 50  0001 C CNN
F 1 "GNDREF" V 13555 7522 50  0000 R CNN
F 2 "" H 13550 7650 50  0001 C CNN
F 3 "" H 13550 7650 50  0001 C CNN
	1    13550 7650
	0    1    1    0   
$EndComp
Text GLabel 14450 7550 2    50   Input ~ 0
SWDIO
Text GLabel 14450 7650 2    50   Input ~ 0
SWCLK
NoConn ~ 13550 7750
NoConn ~ 13550 7850
NoConn ~ 13550 7950
NoConn ~ 14450 7950
NoConn ~ 14450 7850
NoConn ~ 14450 7750
$Comp
L SamacSys_Parts:EVQ-P9W02M S1
U 1 1 6087E09B
P 15400 7850
F 0 "S1" V 15446 7672 50  0000 R CNN
F 1 "EVQ-P9W02M" V 15355 7672 50  0000 R CNN
F 2 "EVQP9W02M" H 15150 7350 50  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Panasonic%20Switches%20PDFs/EVQ-P9_Series_DS.pdf" H 15150 7250 50  0001 L CNN
F 4 "Tactile Switches 4.7x3.5x2.1mm 4N W/O Grnd Term" H 15150 7150 50  0001 L CNN "Description"
F 5 "2.3" H 15150 7050 50  0001 L CNN "Height"
F 6 "667-EVQ-P9W02M" H 15150 6950 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Panasonic/EVQ-P9W02M?qs=PzGy0jfpSMsvy2xbnzB8PQ%3D%3D" H 15150 6850 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 15150 6750 50  0001 L CNN "Manufacturer_Name"
F 9 "EVQ-P9W02M" H 15150 6650 50  0001 L CNN "Manufacturer_Part_Number"
	1    15400 7850
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR021
U 1 1 6089A847
P 15450 7250
F 0 "#PWR021" H 15450 7000 50  0001 C CNN
F 1 "GNDREF" H 15455 7077 50  0000 C CNN
F 2 "" H 15450 7250 50  0001 C CNN
F 3 "" H 15450 7250 50  0001 C CNN
	1    15450 7250
	-1   0    0    1   
$EndComp
NoConn ~ 15200 8650
Wire Wire Line
	15300 8650 15300 8550
Wire Wire Line
	15300 8550 15450 8550
Wire Wire Line
	15450 8550 15450 8450
NoConn ~ 15350 8450
NoConn ~ 15350 7250
$Comp
L XT60-M:XT60-M J2
U 1 1 6091A416
P 15500 5000
F 0 "J2" H 15630 5046 50  0000 L CNN
F 1 "XT60-M" H 15630 4955 50  0000 L CNN
F 2 "AMASS_XT60-M" H 15250 4500 50  0001 L BNN
F 3 "" H 15500 5000 50  0001 L BNN
F 4 "1.2" H 15250 4200 50  0001 L BNN "PARTREV"
F 5 "AMASS" H 15250 4300 50  0001 L BNN "MANUFACTURER"
F 6 "IPC-7251" H 15250 4400 50  0001 L BNN "STANDARD"
	1    15500 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR020
U 1 1 6091BFF0
P 15200 4900
F 0 "#PWR020" H 15200 4650 50  0001 C CNN
F 1 "GNDREF" V 15205 4772 50  0000 R CNN
F 2 "" H 15200 4900 50  0001 C CNN
F 3 "" H 15200 4900 50  0001 C CNN
	1    15200 4900
	0    1    1    0   
$EndComp
Text GLabel 15200 5100 0    50   Input ~ 0
BATT
$Comp
L SamacSys_Parts:ERJ-S02F1002X R24
U 1 1 609B20EC
P 17000 6900
F 0 "R24" H 17150 6800 50  0000 R CNN
F 1 "ERJ-S02F1002X" H 16700 6300 50  0001 L CNN
F 2 "RESC1005X40N" H 16700 6400 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDP0000/AOA0000C334.pdf" H 17550 6850 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 0402 10Kohms 1% Anti-Sulfur" H 16700 6200 50  0001 L CNN "Description"
F 5 "0.4" H 16700 6100 50  0001 L CNN "Height"
F 6 "667-ERJ-S02F1002X" H 16700 6000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Panasonic/ERJ-S02F1002X?qs=Zyl8A9hlmJre8ZnVBnn2RQ%3D%3D" H 16700 5900 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 16700 5800 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-S02F1002X" H 16700 5700 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "10 k" H 17000 6900 50  0000 C CNN "Resistance"
	1    17000 6900
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR026
U 1 1 609B5B5B
P 16650 6900
F 0 "#PWR026" H 16650 6650 50  0001 C CNN
F 1 "GNDREF" H 16655 6727 50  0000 C CNN
F 2 "" H 16650 6900 50  0001 C CNN
F 3 "" H 16650 6900 50  0001 C CNN
	1    16650 6900
	0    1    1    0   
$EndComp
Wire Wire Line
	17600 6900 17350 6900
$Comp
L SamacSys_Parts:ERJ-2RKF7500X R23
U 1 1 60A10881
P 16250 10450
F 0 "R23" H 16150 10550 50  0000 L CNN
F 1 "ERJ-2RKF7500X" H 15950 9950 50  0001 L CNN
F 2 "RESC1005X40N" H 15950 9850 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ-2RKF7500X.pdf" H 15950 9750 50  0001 L CNN
F 4 "Panasonic ERJ-2RK 0402 Resistor Chip" H 15950 9650 50  0001 L CNN "Description"
F 5 "0.4" H 15950 9550 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKF7500X" H 15950 9450 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKF7500X" H 15950 9350 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 15950 9250 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-2RKF7500X" H 15950 9150 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "750" H 16250 10450 50  0000 C CNN "Resistance"
	1    16250 10450
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:APT1608LSECK_J3-PRV LED1
U 1 1 60A12FA5
P 16900 10450
F 0 "LED1" H 16900 10183 50  0000 C CNN
F 1 "APT1608LSECK_J3-PRV" H 16450 9950 50  0001 L BNN
F 2 "APT1608CGCK" H 16450 9850 50  0001 L BNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT1608LSECK-J3-PRV.pdf" H 17100 10500 50  0001 L BNN
F 4 "Standard LEDs - SMD 1.6X0.8MM RED LOW CURRENT SMD" H 16450 9700 50  0001 L BNN "Description"
F 5 "0.75" H 16450 9600 50  0001 L BNN "Height"
F 6 "604-APT1608LSECKJ3RV" H 16450 9500 50  0001 L BNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Kingbright/APT1608LSECK-J3-PRV?qs=6oMev5NRZMFBN%252BbFaEKqAQ%3D%3D" H 16450 9400 50  0001 L BNN "Mouser Price/Stock"
F 8 "Kingbright" H 16450 9300 50  0001 L BNN "Manufacturer_Name"
F 9 "APT1608LSECK/J3-PRV" H 16450 9200 50  0001 L BNN "Manufacturer_Part_Number"
F 10 "Red" H 16900 10274 50  0000 C CNN "Color"
	1    16900 10450
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR028
U 1 1 60A15254
P 17200 10450
F 0 "#PWR028" H 17200 10200 50  0001 C CNN
F 1 "GNDREF" V 17205 10322 50  0000 R CNN
F 2 "" H 17200 10450 50  0001 C CNN
F 3 "" H 17200 10450 50  0001 C CNN
	1    17200 10450
	0    -1   -1   0   
$EndComp
$Comp
L SamacSys_Parts:150060AS75000 LED2
U 1 1 60A170D4
P 19000 6000
F 0 "LED2" V 19046 5870 50  0000 R CNN
F 1 "150060AS75000" H 18700 5500 50  0001 L BNN
F 2 "LEDC1608X80N" H 18700 5400 50  0001 L BNN
F 3 "https://componentsearchengine.com/Datasheets/2/150060AS75000.pdf" H 19200 6050 50  0001 L BNN
F 4 "2.4 V Amber LED 1608 (0603)  SMD, Wurth Elektronik WL-SMCW 150060AS75000" H 18700 5200 50  0001 L BNN "Description"
F 5 "0.8" H 18700 5100 50  0001 L BNN "Height"
F 6 "710-150060AS75000" H 18700 5000 50  0001 L BNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Wurth-Elektronik/150060AS75000/?qs=d0WKAl%252BL4KbvMBYryeof0Q%3D%3D" H 18700 4900 50  0001 L BNN "Mouser Price/Stock"
F 8 "Wurth Elektronik" H 18700 4800 50  0001 L BNN "Manufacturer_Name"
F 9 "150060AS75000" H 18700 4700 50  0001 L BNN "Manufacturer_Part_Number"
F 10 "Amber" V 18955 5870 50  0000 R CNN "Color"
	1    19000 6000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	18900 6500 19000 6500
Wire Wire Line
	19000 6500 19000 6300
$Comp
L SamacSys_Parts:ERJ-2RKF7500X R28
U 1 1 60A1B919
P 19000 5350
F 0 "R28" V 18954 5438 50  0000 L CNN
F 1 "ERJ-2RKF7500X" H 18700 4850 50  0001 L CNN
F 2 "RESC1005X40N" H 18700 4750 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ERJ-2RKF7500X.pdf" H 18700 4650 50  0001 L CNN
F 4 "Panasonic ERJ-2RK 0402 Resistor Chip" H 18700 4550 50  0001 L CNN "Description"
F 5 "0.4" H 18700 4450 50  0001 L CNN "Height"
F 6 "667-ERJ-2RKF7500X" H 18700 4350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=667-ERJ-2RKF7500X" H 18700 4250 50  0001 L CNN "Mouser Price/Stock"
F 8 "Panasonic" H 18700 4150 50  0001 L CNN "Manufacturer_Name"
F 9 "ERJ-2RKF7500X" H 18700 4050 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "750" V 19045 5438 50  0000 L CNN "Resistance"
	1    19000 5350
	0    1    1    0   
$EndComp
Text GLabel 19000 5000 1    50   Input ~ 0
3v3
$Comp
L SamacSys_Parts:TMP235AEDBZRQ1 IC2
U 1 1 60A54464
P 10500 8000
F 0 "IC2" H 10500 8315 50  0000 C CNN
F 1 "TMP235AEDBZRQ1" H 10500 8224 50  0000 C CNN
F 2 "SOT95P237X112-3N" H 10100 7500 50  0001 L CNN
F 3 "https://www.ti.com/lit/gpn/TMP235-Q1" H 10100 7400 50  0001 L CNN
F 4 "Automotive Grade +/-1.5C 2.3V to 5.5V analog output temperature sensor with +10 mV/C gain" H 10100 7300 50  0001 L CNN "Description"
F 5 "1.12" H 10100 7200 50  0001 L CNN "Height"
F 6 "595-TMP235AEDBZRQ1" H 10100 7100 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/TMP235AEDBZRQ1/?qs=T3oQrply3y9acrBXtVPwnw%3D%3D" H 10100 7000 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 10100 6900 50  0001 L CNN "Manufacturer_Name"
F 9 "TMP235AEDBZRQ1" H 10100 6800 50  0001 L CNN "Manufacturer_Part_Number"
	1    10500 8000
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:TMP235AEDBZRQ1 IC3
U 1 1 60A564B7
P 10500 8500
F 0 "IC3" H 10500 8815 50  0000 C CNN
F 1 "TMP235AEDBZRQ1" H 10500 8724 50  0000 C CNN
F 2 "SOT95P237X112-3N" H 10100 8000 50  0001 L CNN
F 3 "https://www.ti.com/lit/gpn/TMP235-Q1" H 10100 7900 50  0001 L CNN
F 4 "Automotive Grade +/-1.5C 2.3V to 5.5V analog output temperature sensor with +10 mV/C gain" H 10100 7800 50  0001 L CNN "Description"
F 5 "1.12" H 10100 7700 50  0001 L CNN "Height"
F 6 "595-TMP235AEDBZRQ1" H 10100 7600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/TMP235AEDBZRQ1/?qs=T3oQrply3y9acrBXtVPwnw%3D%3D" H 10100 7500 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 10100 7400 50  0001 L CNN "Manufacturer_Name"
F 9 "TMP235AEDBZRQ1" H 10100 7300 50  0001 L CNN "Manufacturer_Part_Number"
	1    10500 8500
	1    0    0    -1  
$EndComp
Text GLabel 9950 7950 0    50   Input ~ 0
VBUS
Text GLabel 9950 8450 0    50   Input ~ 0
VBUS
$Comp
L power:GNDREF #PWR05
U 1 1 60A57CA0
P 11050 7950
F 0 "#PWR05" H 11050 7700 50  0001 C CNN
F 1 "GNDREF" V 11055 7822 50  0000 R CNN
F 2 "" H 11050 7950 50  0001 C CNN
F 3 "" H 11050 7950 50  0001 C CNN
	1    11050 7950
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR06
U 1 1 60A58B9C
P 11050 8450
F 0 "#PWR06" H 11050 8200 50  0001 C CNN
F 1 "GNDREF" V 11055 8322 50  0000 R CNN
F 2 "" H 11050 8450 50  0001 C CNN
F 3 "" H 11050 8450 50  0001 C CNN
	1    11050 8450
	0    -1   -1   0   
$EndComp
Text GLabel 9250 8050 0    50   Input ~ 0
TEMP1
Text GLabel 9250 8550 0    50   Input ~ 0
TEMP2
Text GLabel 14100 10350 0    50   Input ~ 0
TEMP2
Text GLabel 14100 10050 0    50   Input ~ 0
TEMP1
Wire Wire Line
	19350 6800 18900 6800
Wire Wire Line
	19350 7300 19800 7300
Connection ~ 19800 7300
Wire Wire Line
	19800 7300 20350 7300
Connection ~ 20350 7300
Wire Wire Line
	19350 6800 19800 6800
Connection ~ 19350 6800
Connection ~ 19800 6800
Wire Wire Line
	19900 6800 20350 6800
$Comp
L SamacSys_Parts:06033C470KAT2A C14
U 1 1 60ADA948
P 19350 8250
F 0 "C14" H 19350 8550 50  0000 C CNN
F 1 "06033C470KAT2A" H 19050 7750 50  0001 L CNN
F 2 "CAPC1608X90N" H 19050 7650 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06033C470KAT2A.pdf" H 19050 7550 50  0001 L CNN
F 4 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 25V 47pF X7R 0603 10% Tol" H 19050 7450 50  0001 L CNN "Description"
F 5 "0.9" H 19050 7350 50  0001 L CNN "Height"
F 6 "581-06033C470KAT2A" H 19050 7250 50  0001 L CNN "Mouser Part Number"
F 7 "http://www.mouser.com/Search/ProductDetail.aspx?qs=zRMn1EiLHUce0mcQKVYXJg%3d%3d" H 19050 7150 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 19050 7050 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C470KAT2A" H 19050 6950 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "47 pF" H 19350 8450 50  0000 C CNN "Capacitance"
	1    19350 8250
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:06033C470KAT2A C13
U 1 1 60ADAA1B
P 19350 7650
F 0 "C13" H 19350 7385 50  0000 C CNN
F 1 "06033C470KAT2A" H 19050 7150 50  0001 L CNN
F 2 "CAPC1608X90N" H 19050 7050 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/06033C470KAT2A.pdf" H 19050 6950 50  0001 L CNN
F 4 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 25V 47pF X7R 0603 10% Tol" H 19050 6850 50  0001 L CNN "Description"
F 5 "0.9" H 19050 6750 50  0001 L CNN "Height"
F 6 "581-06033C470KAT2A" H 19050 6650 50  0001 L CNN "Mouser Part Number"
F 7 "http://www.mouser.com/Search/ProductDetail.aspx?qs=zRMn1EiLHUce0mcQKVYXJg%3d%3d" H 19050 6550 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 19050 6450 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C470KAT2A" H 19050 6350 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "47 pF" H 19350 7476 50  0000 C CNN "Capacitance"
	1    19350 7650
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:47346-0001 J4
U 1 1 6048A625
P 21050 8000
F 0 "J4" H 21250 8350 50  0000 C CNN
F 1 "47346-0001" H 20800 7400 50  0001 L CNN
F 2 "47346-0001" H 20800 7500 50  0001 L CNN
F 3 "http://www.molex.com/pdm_docs/sd/473460001_sd.pdf" H 20800 7400 50  0001 L CNN
F 4 "Micro USB B Receptacle Bottom Mount Assy Molex Right Angle SMT Type B Version 2.0 Micro USB Connector Socket, 30 V ac, 1A 47352 MICRO-USB" H 20800 7300 50  0001 L CNN "Description"
F 5 "538-47346-0001" H 20800 7100 50  0001 L CNN "Mouser Part Number"
F 6 "https://www.mouser.co.uk/ProductDetail/Molex/47346-0001/?qs=c2CV6XM0DweJBWaSeyWeCw%3D%3D" H 20800 7000 50  0001 L CNN "Mouser Price/Stock"
F 7 "Molex" H 20800 6900 50  0001 L CNN "Manufacturer_Name"
F 8 "47346-0001" H 20800 6800 50  0001 L CNN "Manufacturer_Part_Number"
	1    21050 8000
	-1   0    0    -1  
$EndComp
Connection ~ 18500 7400
Wire Wire Line
	19600 7650 19700 7650
Wire Wire Line
	19700 7650 19700 7900
Wire Wire Line
	19600 8250 19700 8250
Wire Wire Line
	19700 8250 19700 8000
Wire Wire Line
	20600 7900 19700 7900
Connection ~ 19700 7900
Wire Wire Line
	19700 8000 20600 8000
Connection ~ 19700 8000
$Comp
L power:GNDREF #PWR034
U 1 1 60B042F8
P 19100 8250
F 0 "#PWR034" H 19100 8000 50  0001 C CNN
F 1 "GNDREF" V 19105 8122 50  0000 R CNN
F 2 "" H 19100 8250 50  0001 C CNN
F 3 "" H 19100 8250 50  0001 C CNN
	1    19100 8250
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR033
U 1 1 60B0527C
P 19100 7650
F 0 "#PWR033" H 19100 7400 50  0001 C CNN
F 1 "GNDREF" V 19105 7522 50  0000 R CNN
F 2 "" H 19100 7650 50  0001 C CNN
F 3 "" H 19100 7650 50  0001 C CNN
	1    19100 7650
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:06033C104KAT2A C1
U 1 1 60B45125
P 9000 5500
F 0 "C1" V 8954 5628 50  0000 L CNN
F 1 "06033C104KAT2A" H 8700 4900 50  0001 L CNN
F 2 "CAPC1608X90N" H 8700 5000 50  0001 L CNN
F 3 "" H 8700 4900 50  0001 L CNN
F 4 "Capacitor MLCC 0603 100nF 25V AVX 0603 Standard 100nF Ceramic Multilayer Capacitor, 25 V dc, +125C, X7R Dielectric, +/-10% SMD" H 8700 4800 50  0001 L CNN "Description"
F 5 "0.9" H 8700 4700 50  0001 L CNN "Height"
F 6 "581-06033C104KAT2A" H 8700 4600 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/AVX/06033C104KAT2A/?qs=gPDEucxdFwYuZhv3uXRdIw%3D%3D" H 8700 4500 50  0001 L CNN "Mouser Price/Stock"
F 8 "AVX" H 8700 4400 50  0001 L CNN "Manufacturer_Name"
F 9 "06033C104KAT2A" H 8700 4300 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "100 nF" V 9045 5628 50  0000 L CNN "Capacitance"
	1    9000 5500
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR03
U 1 1 60B47ED7
P 9000 5750
F 0 "#PWR03" H 9000 5500 50  0001 C CNN
F 1 "GNDREF" H 9005 5577 50  0000 C CNN
F 2 "" H 9000 5750 50  0001 C CNN
F 3 "" H 9000 5750 50  0001 C CNN
	1    9000 5750
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:MBR120VLSFT3G D2
U 1 1 60B49947
P 18100 9250
F 0 "D2" V 18054 9380 50  0000 L CNN
F 1 "MBR120VLSFT3G" V 18145 9380 50  0000 L CNN
F 2 "SODFL3616X98N" H 17800 8750 50  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/MBR120VLSFT1-D.PDF" H 17800 8650 50  0001 L CNN
F 4 "ON SEMICONDUCTOR - MBR120VLSFT3G - SCHOTTKY RECT, 1A, 20V, SOD123" H 17800 8550 50  0001 L CNN "Description"
F 5 "0.98" H 17800 8450 50  0001 L CNN "Height"
F 6 "863-MBR120VLSFT3G" H 17800 8350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/ON-Semiconductor/MBR120VLSFT3G/?qs=3JMERSakebrD2yr9jFliaA%3D%3D" H 17800 8250 50  0001 L CNN "Mouser Price/Stock"
F 8 "ON Semiconductor" H 17800 8150 50  0001 L CNN "Manufacturer_Name"
F 9 "MBR120VLSFT3G" H 17800 8050 50  0001 L CNN "Manufacturer_Part_Number"
	1    18100 9250
	0    1    1    0   
$EndComp
$Comp
L SamacSys_Parts:MBR120VLSFT3G D3
U 1 1 60B4D367
P 20300 7800
F 0 "D3" H 20300 8067 50  0000 C CNN
F 1 "MBR120VLSFT3G" H 20300 7976 50  0000 C CNN
F 2 "SODFL3616X98N" H 20000 7300 50  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/MBR120VLSFT1-D.PDF" H 20000 7200 50  0001 L CNN
F 4 "ON SEMICONDUCTOR - MBR120VLSFT3G - SCHOTTKY RECT, 1A, 20V, SOD123" H 20000 7100 50  0001 L CNN "Description"
F 5 "0.98" H 20000 7000 50  0001 L CNN "Height"
F 6 "863-MBR120VLSFT3G" H 20000 6900 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/ON-Semiconductor/MBR120VLSFT3G/?qs=3JMERSakebrD2yr9jFliaA%3D%3D" H 20000 6800 50  0001 L CNN "Mouser Price/Stock"
F 8 "ON Semiconductor" H 20000 6700 50  0001 L CNN "Manufacturer_Name"
F 9 "MBR120VLSFT3G" H 20000 6600 50  0001 L CNN "Manufacturer_Part_Number"
	1    20300 7800
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:MBR120VLSFT3G D1
U 1 1 60B56654
P 15200 5400
F 0 "D1" V 15246 5270 50  0000 R CNN
F 1 "MBR120VLSFT3G" V 15155 5270 50  0000 R CNN
F 2 "SODFL3616X98N" H 14900 4900 50  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/MBR120VLSFT1-D.PDF" H 14900 4800 50  0001 L CNN
F 4 "ON SEMICONDUCTOR - MBR120VLSFT3G - SCHOTTKY RECT, 1A, 20V, SOD123" H 14900 4700 50  0001 L CNN "Description"
F 5 "0.98" H 14900 4600 50  0001 L CNN "Height"
F 6 "863-MBR120VLSFT3G" H 14900 4500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/ON-Semiconductor/MBR120VLSFT3G/?qs=3JMERSakebrD2yr9jFliaA%3D%3D" H 14900 4400 50  0001 L CNN "Mouser Price/Stock"
F 8 "ON Semiconductor" H 14900 4300 50  0001 L CNN "Manufacturer_Name"
F 9 "MBR120VLSFT3G" H 14900 4200 50  0001 L CNN "Manufacturer_Part_Number"
	1    15200 5400
	0    -1   -1   0   
$EndComp
Text GLabel 15200 5700 3    50   Output ~ 0
VBUS
$Comp
L SamacSys_Parts:RC0402FR-075KL R4
U 1 1 607A3987
P 9600 8050
F 0 "R4" H 9700 7950 50  0000 C CNN
F 1 "RC0402FR-075KL" H 9300 7450 50  0001 L CNN
F 2 "RESC1005X40N" H 9300 7550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC0402_51_RoHS_L_6_r.pdf" H 9300 7450 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 5K ohm 1% 50V General Purpose" H 9300 7350 50  0001 L CNN "Description"
F 5 "0.4" H 9300 7250 50  0001 L CNN "Height"
F 6 "603-RC0402FR-075KL" H 9300 7150 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Yageo/RC0402FR-075KL/?qs=9h0bZHM%2F3zL5adZvfm%252BR4Q%3D%3D" H 9300 7050 50  0001 L CNN "Mouser Price/Stock"
F 8 "YAGEO (PHYCOMP)" H 9300 6950 50  0001 L CNN "Manufacturer_Name"
F 9 "RC0402FR-075KL" H 9300 6850 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "5 k" H 9600 8050 50  0000 C CNN "Resistance"
	1    9600 8050
	-1   0    0    1   
$EndComp
$Comp
L SamacSys_Parts:RC0402FR-075KL R5
U 1 1 607A460E
P 9600 8550
F 0 "R5" H 9700 8450 50  0000 C CNN
F 1 "RC0402FR-075KL" H 9300 7950 50  0001 L CNN
F 2 "RESC1005X40N" H 9300 8050 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC0402_51_RoHS_L_6_r.pdf" H 9300 7950 50  0001 L CNN
F 4 "Thick Film Resistors - SMD 5K ohm 1% 50V General Purpose" H 9300 7850 50  0001 L CNN "Description"
F 5 "0.4" H 9300 7750 50  0001 L CNN "Height"
F 6 "603-RC0402FR-075KL" H 9300 7650 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Yageo/RC0402FR-075KL/?qs=9h0bZHM%2F3zL5adZvfm%252BR4Q%3D%3D" H 9300 7550 50  0001 L CNN "Mouser Price/Stock"
F 8 "YAGEO (PHYCOMP)" H 9300 7450 50  0001 L CNN "Manufacturer_Name"
F 9 "RC0402FR-075KL" H 9300 7350 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "5 k" H 9600 8550 50  0000 C CNN "Resistance"
	1    9600 8550
	-1   0    0    1   
$EndComp
NoConn ~ 14400 11350
Wire Bus Line
	18500 7400 18500 7900
$EndSCHEMATC
