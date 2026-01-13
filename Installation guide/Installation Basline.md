# Installation Baseline — Pearl Island Weather Station
Date: 2026-01-13

## Purpose
This document captures the known-good configuration of the Pearl Island
weather station and base station at the time of island installation.

## Git Reference
- Repo: WeatherStations
- Branch: main
- Tag: install-2026-01-13
- Hash: 88afc0a

## Island Station (Pearl)
- Hardware:
  - Gill WindSonic 75 (RS-422 NMEA)
  - Heltec WiFi LoRa 32 V3
  - Laird OD9-5 omni antenna
- Power:
  - Solar + LiFePO4 battery
  - Nominal battery voltage ~13.2 V
- Firmware:
  - TX interval: 5 minutes
  - Sampling: 1 Hz
  - Wind speed: 2-min average
  - Direction: 2-min vector average
  - TX power: 14
  - Frequency: 915 MHz

## Base Station
- Platform: ESP32
- Function:
  - Receives LoRa packets
  - Posts to Google Sheets "crescent_data" sheet "Pearl" via apps script
  
- Network:
  - Wi-Fi dependent
  - Internet outages expected to coincide with power outages

## Known Quirks
- Battery ADC reads ~1 V lower than physical measurement
- Blank rows may appear on startup
- Direction accuracy prioritized less than speed

## Recovery / Troubleshooting
- If no data appears online:
  1. Check base station power
  2. Check LoRa RX counters
  3. Verify sheet timestamps advancing
- If LoRa stops:
  - Inspect antenna and coax for corrosion
