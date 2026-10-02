EESchema Schematic File Version 4
LIBS:pearl_v2_final-cache
EELAYER 29 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Pearl V2 — Final Intended Circuit"
Date "2026-10-01"
Rev "Audit 1"
Comp "Pearl Island Weather Station"
Comment1 "Schematic only — no PCB design"
Comment2 "V2 breadboard circuit plus proven retained V1 protection"
Comment3 "Source: current firmware, V1/V2 photos, project records, manufacturer data"
Comment4 "Unestablished details are explicitly noted; no substitutions"
$EndDescr
Text Notes 700 650 0 120 ~ 24
FINAL PEARL V2 SCHEMATIC — SCHEMATIC ONLY / NO PCB
Text Notes 700 900 0 75 ~ 15
Proven V2 breadboard circuitry is combined with proven retained V1 input protection. Provenance is marked per section.
Text Notes 700 1200 0 90 ~ 18
PROVEN RETAINED V1 POWER / PROTECTION
$Comp
L PEARL_BATTERY_INPUT J1
U 1 1 1001
P 1300 1650
F 0 "J1" H 1300 2000 50 0000 C CNN
F 1 "12 V BATTERY INPUT" H 1300 1300 50 0000 C CNN
	1    1300 1650
	1 0 0 -1
$EndComp
$Comp
L PEARL_D_SCHOTTKY D1
U 1 1 1002
P 2450 1550
F 0 "D1" H 2450 1775 50 0000 C CNN
F 1 "1N5822" H 2450 1325 50 0000 C CNN
	1    2450 1550
	1 0 0 -1
$EndComp
$Comp
L PEARL_D_TVS D2
U 1 1 1003
P 3100 1900
F 0 "D2" H 3300 1925 50 0000 L CNN
F 1 "P6KE16A" H 3300 1825 50 0000 L CNN
	1    3100 1900
	1 0 0 -1
$EndComp
$Comp
L PEARL_OKI78SR U1
U 1 1 1004
P 4450 1650
F 0 "U1" H 4450 2050 50 0000 C CNN
F 1 "OKI-78SR-5/1.5-W36H-C" H 4450 1250 50 0000 C CNN
	1    4450 1650
	1 0 0 -1
$EndComp
Wire Wire Line
	1800 1550 2250 1550
Wire Wire Line
	2650 1550 3850 1550
Wire Wire Line
	3100 1550 3100 1700
Connection ~ 3100 1550
Wire Wire Line
	4450 2150 4450 2300
Wire Wire Line
	1800 1750 1800 2300
Wire Wire Line
	1800 2300 4450 2300
Wire Wire Line
	3100 2100 3100 2300
Connection ~ 3100 2300
Wire Wire Line
	5050 1550 5650 1550
Text Label 1850 1550 0 50 ~ 0
BATT_RAW_12V
Text Label 2700 1550 0 50 ~ 0
PROTECTED_12V
Text Label 5150 1550 0 50 ~ 0
+5V
Text Label 1900 2300 0 50 ~ 0
GND
Text Notes 2850 1350 0 50 ~ 10
D1 band/cathode toward PROTECTED_12V
Text Notes 3250 2150 0 50 ~ 10
D2 unidirectional: cathode to PROTECTED_12V, anode to GND
Text Notes 3950 2450 0 50 ~ 10
U1 pins: 1=+VIN, 2=GND, 3=+5V
Text Notes 700 2850 0 90 ~ 18
PROVEN V2 BREADBOARD — BATTERY DIVIDER / ADC FILTER
$Comp
L PEARL_R R1
U 1 1 1101
P 2450 3300
F 0 "R1" H 2450 3400 50 0000 C CNN
F 1 "4.7 kΩ" H 2450 3200 50 0000 C CNN
	1    2450 3300
	1 0 0 -1
$EndComp
$Comp
L PEARL_R R2
U 1 1 1102
P 3300 3850
F 0 "R2" H 3300 3950 50 0000 C CNN
F 1 "1 kΩ" H 3300 3750 50 0000 C CNN
	1    3300 3850
	0 1 1 0
$EndComp
$Comp
L PEARL_C_POL C1
U 1 1 1103
P 4050 3850
F 0 "C1" H 4200 3925 50 0000 L CNN
F 1 "1 µF electrolytic" H 4200 3825 50 0000 L CNN
	1    4050 3850
	1 0 0 -1
$EndComp
$Comp
L PEARL_R R3
U 1 1 1104
P 5000 3300
F 0 "R3" H 5000 3400 50 0000 C CNN
F 1 "100 Ω" H 5000 3200 50 0000 C CNN
	1    5000 3300
	1 0 0 -1
$EndComp
$Comp
L PEARL_C C2
U 1 1 1105
P 5800 3850
F 0 "C2" H 5950 3925 50 0000 L CNN
F 1 "0.1 µF ceramic" H 5950 3825 50 0000 L CNN
	1    5800 3850
	1 0 0 -1
$EndComp
Wire Wire Line
	2050 3300 2250 3300
Wire Wire Line
	2650 3300 4800 3300
Wire Wire Line
	3300 3300 3300 3650
Wire Wire Line
	4050 3300 4050 3650
Connection ~ 3300 3300
Connection ~ 4050 3300
Wire Wire Line
	5200 3300 6500 3300
Wire Wire Line
	5800 3300 5800 3650
Connection ~ 5800 3300
Wire Wire Line
	3300 4050 3300 4250
Wire Wire Line
	4050 4050 4050 4250
Wire Wire Line
	5800 4050 5800 4250
Wire Wire Line
	3300 4250 5800 4250
Text Label 2050 3300 0 50 ~ 0
PROTECTED_12V
Text Label 2850 3300 0 50 ~ 0
DIVIDER_NODE
Text Label 5350 3300 0 50 ~ 0
ADC_NODE
Text Label 3450 4250 0 50 ~ 0
GND
Text Notes 3600 4450 0 50 ~ 10
C1 black electrolytic: positive to DIVIDER_NODE, negative to GND
Text Notes 5350 4450 0 50 ~ 10
C2 orange ceramic disc
$Comp
L PEARL_HELTEC_V4 U3
U 1 1 1201
P 9200 3900
F 0 "U3" H 9200 4925 60 0000 C CNN
F 1 "Heltec WiFi LoRa 32 V4" H 9200 2875 60 0000 C CNN
	1    9200 3900
	1 0 0 -1
$EndComp
Wire Wire Line
	6500 3300 6500 4100
Wire Wire Line
	6500 4100 8300 4100
Wire Wire Line
	8000 3300 8300 3300
Wire Wire Line
	8000 3500 8300 3500
Text Label 8000 3300 0 50 ~ 0
+5V
Text Label 8000 3500 0 50 ~ 0
GND
Text Label 6900 4100 0 50 ~ 0
ADC_NODE
Text Notes 10150 4500 0 50 ~ 10
USB-C is the existing programming/serial diagnostic connection.
Text Notes 10150 4625 0 50 ~ 10
OLED and GPIO35 heartbeat LED are onboard diagnostic devices.
$Comp
L PEARL_RF_INTERCONNECT J3
U 1 1 1202
P 10800 3300
F 0 "J3" H 10800 3550 50 0000 C CNN
F 1 "IPEX1.0-to-SMA bulkhead pigtail" H 10800 3050 50 0000 C CNN
	1    10800 3300
	1 0 0 -1
$EndComp
$Comp
L PEARL_ANTENNA AE1
U 1 1 1203
P 11900 3300
F 0 "AE1" H 11900 3700 50 0000 C CNN
F 1 "Laird OD9-5" H 11900 2900 50 0000 C CNN
	1    11900 3300
	1 0 0 -1
$EndComp
Wire Wire Line
	10100 3300 10400 3300
Wire Wire Line
	11200 3300 11600 3300
Text Label 10300 3300 0 50 ~ 0
LORA_RF
Text Notes 10400 2700 0 50 ~ 10
RF path is through the Heltec onboard IPEX1.0 LoRa connector.
Text Notes 10400 2800 0 50 ~ 10
Field build is 915 MHz; lab build is 914 MHz. Both use the retained 915 MHz antenna.
Text Notes 10400 2900 0 50 ~ 10
Base-station connection is RF only; there is no wired Pearl/base net.
Text Notes 700 5200 0 90 ~ 18
PROVEN WINDSONIC / RS-422 INPUT PATH
$Comp
L PEARL_WINDSONIC J2
U 1 1 1301
P 1650 6900
F 0 "J2" H 1650 8050 60 0000 C CNN
F 1 "Gill WindSonic 75 — 9-way circular" H 1650 5750 60 0000 C CNN
	1    1650 6900
	1 0 0 -1
$EndComp
$Comp
L PEARL_WAVESHARE_RS422 U2
U 1 1 1302
P 5350 6900
F 0 "U2" H 5350 8050 60 0000 C CNN
F 1 "Waveshare TTL TO RS422 (B), SKU 23652" H 5350 5750 60 0000 C CNN
	1    5350 6900
	1 0 0 -1
$EndComp
Wire Wire Line
	2400 6200 3100 6200
Text Label 2450 6200 0 50 ~ 0
WIND_SIGNAL_GND
Wire Wire Line
	4000 6700 4400 6700
Text Label 4000 6700 0 50 ~ 0
WIND_SIGNAL_GND
Wire Wire Line
	2400 6400 3100 6400
Text Label 2450 6400 0 50 ~ 0
PROTECTED_12V
Wire Wire Line
	2400 6600 3100 6600
Text Label 2450 6600 0 50 ~ 0
GND
Wire Wire Line
	2400 6800 3300 6800
Text Label 2500 6800 0 50 ~ 0
WIND_TXD+
Wire Wire Line
	4000 6300 4400 6300
Text Label 4000 6300 0 50 ~ 0
WIND_TXD+
Wire Wire Line
	2400 7000 3300 7000
Text Label 2500 7000 0 50 ~ 0
WIND_TXD-
Wire Wire Line
	4000 6500 4400 6500
Text Label 4000 6500 0 50 ~ 0
WIND_TXD-
NoConn ~ 2400 7200
NoConn ~ 2400 7400
NoConn ~ 2400 7600
NoConn ~ 2400 7800
NoConn ~ 4400 6900
NoConn ~ 4400 7100
Wire Wire Line
	6300 6300 7100 6300
Text Label 6450 6300 0 50 ~ 0
WIND_UART_RX_GPIO7
Wire Wire Line
	7900 3900 8300 3900
Text Label 7900 3900 0 50 ~ 0
WIND_UART_RX_GPIO7
Wire Wire Line
	6300 6500 6900 6500
Text Label 6450 6500 0 50 ~ 0
+5V
Wire Wire Line
	6300 6700 6900 6700
Wire Wire Line
	6300 7100 6900 7100
Wire Wire Line
	6900 6700 6900 7100
Text Label 6450 6700 0 50 ~ 0
GND
NoConn ~ 6300 6900
Text Notes 7600 6225 0 50 ~ 10
Proven breadboard wiring: U2 RXD to GPIO7 / J3.18 via purple jumper; 4800 baud, SERIAL_8N1.
Text Notes 7600 6350 0 50 ~ 10
AUDIT CONFLICT: current src/main.cpp defines WIND_RX_PIN=19. Firmware was not modified.
Text Notes 7600 6475 0 50 ~ 10
U2 RXD is the TTL receiver output; U2 TXD and differential T+/T- are unused.
Text Notes 7600 6600 0 50 ~ 10
U2 internal optional 120 Ω termination jumper state is not visible in project evidence.
Text Notes 700 8500 0 90 ~ 18
FINAL PEARL V2 = PROVEN V2 BREADBOARD + PROVEN RETAINED V1 CIRCUITRY
Text Notes 700 8775 0 55 ~ 10
Retained V1: D1 1N5822 reverse-polarity series diode, D2 P6KE16A shunt TVS, and deployed WindSonic/RS-422 module path.
Text Notes 700 8900 0 55 ~ 10
V2 breadboard: Murata 12 V-to-5 V regulator, exact R1/R2/R3 + C1/C2 ADC network, ADC GPIO2, physical WindSonic GPIO7, OLED and LoRa diagnostics.
Text Notes 700 9200 0 65 ~ 13
UNRESOLVED BUT NOT INVENTED
Text Notes 700 9425 0 50 ~ 10
Connector manufacturer part numbers, resistor tolerances/wattages, capacitor voltage ratings, RS-422 termination-jumper state, and RF pigtail part number are not established.
Text Notes 700 9550 0 50 ~ 10
WindSonic option number and the functions of its unused pins 8/9 are not established. Exact protection-part values inside the purchased U2 module are not expanded.
Text Notes 700 9675 0 50 ~ 10
STOP CONDITION: proven purple-jumper wiring is GPIO7/J3.18, but current firmware source selects GPIO19/J2.18. Reconcile before PCB design.
Text Notes 700 9750 0 50 ~ 10
Except for the specifically identified purple jumper, other wire colors are not treated as pin identity.
Text Notes 700 9825 0 65 ~ 13
NO PCB, FOOTPRINT SELECTION, OR PCB-SPECIFIC PROTECTION CHANGES ARE INCLUDED.
$EndSCHEMATC
