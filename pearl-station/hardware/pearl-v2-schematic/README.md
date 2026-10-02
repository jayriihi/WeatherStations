# Pearl V2 final schematic audit

This directory contains the schematic-only record for the intended final Pearl V2. No PCB or footprint design is included.

## Files

- `pearl_v2_final.sch` — KiCad legacy schematic, importable by current KiCad versions.
- `pearl_v2_final-cache.lib` — self-contained symbols used by the schematic.
- `pearl_v2_bom.csv` — component/BOM summary with validation provenance.

## Provenance classes

The schematic distinguishes:

1. **Proven V2 breadboard circuit** — Murata 5 V regulator, Heltec V4, exact ADC divider/filter, GPIO2, physically wired WindSonic GPIO7, onboard diagnostics, and LoRa configuration.
2. **Proven retained V1 circuitry** — 1N5822 series Schottky, P6KE16A shunt TVS, deployed WindSonic interface hardware, and retained antenna.
3. **Final Pearl V2** — the union of those two proven groups.

## Electrical decisions represented

- `BATT_RAW_12V -> D1 1N5822 -> PROTECTED_12V`.
- D1 anode is toward the battery; its cathode/banded end is toward `PROTECTED_12V`.
- D2 P6KE16A is unidirectional, with cathode on `PROTECTED_12V` and anode on ground.
- U1 pin 1 is `PROTECTED_12V`, pin 2 is ground, and pin 3 is `+5V`.
- The WindSonic and battery divider are supplied from `PROTECTED_12V`.
- Heltec J2.2 receives `+5V`; J2.1 is ground.
- ADC topology is exactly:
  - `PROTECTED_12V -> R1 4.7k -> DIVIDER_NODE`
  - `DIVIDER_NODE -> R2 1k -> GND`
  - `DIVIDER_NODE -> C1 1uF -> GND`, positive at the node
  - `DIVIDER_NODE -> R3 100R -> ADC_NODE`
  - `ADC_NODE -> C2 0.1uF -> GND`
  - `ADC_NODE -> Heltec GPIO2 / J3.13`
- WindSonic receive path is `WindSonic TXD+/- -> Waveshare R+/- -> module RXD -> Heltec GPIO7 / J3.18` through the user-identified purple jumper.
- The RS-422 transmit path is unused because current firmware sets UART TX to `-1`.
- Pearl-to-base communication is LoRa RF only; there is no wired base connection.
- The retained RF path is the Heltec LoRa IPEX1.0 port, an IPEX1.0-to-SMA bulkhead pigtail, and the Laird OD9-5 antenna. The pigtail manufacturer part number is not established.

## Sources checked

- Current `pearl-station/src/main.cpp`, V4 board definition, and V4 Arduino variant.
- Current V4 breadboard photographs supplied 2026-10-01.
- V1 installation photographs, hand diagram, and installation baseline.
- WindSonic generator documentation and physical-input test project.
- Murata OKI-78SR manufacturer datasheet.
- Microchip 1N5822 manufacturer data.
- Littelfuse P6KE series manufacturer datasheet.
- Heltec WiFi LoRa 32 V4 official datasheet.
- Gill WindSonic official manual.
- Waveshare TTL TO RS422 (B) official interface description.

The supplied 2026-10-01 component screenshots are authoritative for U1 (`OKI-78SR-5/1.5-W36H-C`), D1 (`1N5822`), and D2 (`P6KE16A`). The user-supplied breadboard topology is authoritative for R1-R3 and C1-C2. The V1 installation photos establish the retained Waveshare converter and IPEX-to-SMA bulkhead RF path.

## Stop condition: physical wiring and firmware source conflict

The additional board photograph and user identification establish that the purple jumper carrying U2 RXD is physically connected to **GPIO7, Heltec header J3 pin 18**. The current `pearl-station/src/main.cpp` instead defines `WIND_RX_PIN` as **GPIO19, Heltec header J2 pin 18**. The schematic records the proven physical wiring on GPIO7 and flags this discrepancy; the firmware has not been changed. This conflict must be reconciled before PCB design.

Manufacturer references used for pinouts and established ratings:

- [Murata OKI-78SR datasheet](https://www.murata.com/products/productdata/8807037992990/oki-78sr.pdf)
- [Microchip 1N5822 datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/HRDS/ProductDocuments/DataSheets/1N5822-DSB5820-5822.pdf)
- [Littelfuse P6KE series datasheet](https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_p6ke_datasheet.pdf.pdf)
- [Heltec WiFi LoRa 32 V4 datasheet](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/datasheet/WiFi_LoRa_32_V4.2.0.pdf)
- [Gill WindSonic manual](https://gillinstruments.com/wp-content/uploads/2025/01/1405-PS-0019-WindSonic-Manual.pdf)
- [Waveshare TTL TO RS422 (B)](https://www.waveshare.com/ttl-to-rs422-b.htm)

## Explicit unresolved details

These have not been guessed:

- Manufacturer part numbers for the external battery and field wiring connectors.
- Resistor tolerances and power ratings.
- C1 and C2 voltage ratings.
- The internal 120-ohm termination-jumper state in the Waveshare module.
- The exact RF pigtail and panel connector part numbers.
- The exact orderable Heltec V4 hardware suffix. The evidence establishes a WiFi LoRa 32 V4 operating at 914/915 MHz, not which 863-928 MHz ordering variant was purchased.
- The exact WindSonic option number and therefore the functions of unused sensor pins 8 and 9; both pins are explicitly unused by Pearl.
- Exact part numbers and values for protection devices internal to the purchased Waveshare converter module. The converter is represented as a complete, proven module rather than reverse-engineered.
- The GPIO7 physical wiring versus GPIO19 current-source discrepancy described above.
- Except for the specifically identified purple jumper, physical wire colors are not used as electrical identifiers.

Open `pearl_v2_final.sch` in KiCad and allow KiCad to convert it to the current `.kicad_sch` format when prompted. Preserve the legacy source until the converted file has been visually checked.
