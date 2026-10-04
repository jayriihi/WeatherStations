# Pearl V2 provisional component-side placement

Review preview only. Electrical design unchanged. Origin is the upper-left corner of the component-side view; columns increase right and rows increase down. Hole (C,R) is at (2.54C, 2.54R) mm. No files outside this preview directory were changed by the drawing script.

Board: ST-PERF-2-3, 76.20 × 50.80 mm; 0.040-inch component holes. Mounting centres: (2.54,2.54), (73.66,2.54), (2.54,48.26), (73.66,48.26) mm; diameter 3.175 mm. The orange Ø9 mm hardware clearances are provisional design allowances. Corner omissions follow the manufacturer drawing.

Heltec: USB right, display upward, J2 upper and J3 lower, pin 1 at right. Header pitch is 2.54 mm and row separation 22.86 mm. Drawn body 51.69 × 25.40 mm at x=3.81, y=8.89 mm; longitudinal envelope registration remains provisional pending complete mechanical callout review. Socket bodies, mounting height and underside clearance are TBD. This drawing does not assign extra Heltec mounting screws.

Murata: exact horizontal H model retained. Drawn 10.4 × 16.5 mm nominal body; body-to-pin offset provisional. Manufacturer dimensional tolerances and physical assembly clearances must be checked before build release.

D1 is the retained Microchip 1N5822, not a substituted generic diode. Its body marker is illustrative; the formed lead span is provisional. Proven Pearl V1 procedure: lightly sand/reduce leads only if necessary to fit the perfboard holes, remove sanding debris, and insert without force. Cathode/band faces PROTECTED_12V.

R1–R3 and C1–C2: proposed hole locations only. Exact manufacturer packages, body dimensions and natural lead pitches remain TBD; the drawn markers are not component-size claims. C1 positive is DIVIDER_NODE.

Authoritative physical arrangement supplied by the user, 04 October 2026: exactly SIX separate board wire landings. Wires 1–2: battery 12V+ and 12V− into Pearl. Wire 3: protected 12V+ to WindSonic. WindSonic supply− / J2.3 connects DIRECTLY to battery negative off the Pearl board; it has no Pearl wire landing. Wires 4–6: TXD (user harness label), GND and VCC (+5V) to Waveshare. Battery return and Waveshare return are separate physical PCB wires despite sharing GND. Former C26,R1 sensor-return and C9,R19 extra-module-ground landings are unassigned. This preserves the electrical GND membership in the engineering schematic.

WORKING HARDWARE AUTHORITY: Waveshare TXD is USED and sends data over the purple TXD wire to Heltec GPIO6, at the physical underside header hole marked “6”. Waveshare RXD is UNUSED. The assembler-facing connection is Waveshare TXD → TXD wire → Heltec GPIO6 (pin marked “6” on board). The existing board net name WIND_UART_RX_GPIO6 and engineering cross-reference J3.17 are retained; receive refers to the Heltec role. The physical terminal identification is supplied by the user. No engineering schematic, firmware or PCB edits are made.

External connections: battery ± and sensor supply+ leave the upper edge. Sensor supply− goes directly to battery negative, not through a Pearl landing. Waveshare TXD harness wire, GND and +5V VCC leave the lower edge in left-to-right order TXD → GND → VCC: TXD at C5,R19 (12.70,48.26 mm), GND at C7,R19 (17.78,48.26 mm), VCC at C11,R19 (27.94,48.26 mm). Only TXD and VCC wire landings were exchanged; all component positions and electrical nets are unchanged. The schedule’s U2.GND1/GND2 describe electrical ground membership, not two board-to-module conductors; one physical ground lead leaves Pearl as instructed. WindSonic pin 4 → Waveshare R+, pin 5 → R−, pin 1 → RGND; these bypass the perfboard. Unused Waveshare RXD, T+, T− and sensor pins 6–9 remain unwired. Signal ground is distinct from logic/battery ground. Heltec LoRa IPEX → SMA bulkhead pigtail → antenna leaves toward the left; no perfboard RF pad is added. The six-wire count excludes that coax and temporary USB. Waveshare remains external on its DIN rail.

The sole TXD external wire runs from Waveshare TXD to the C5,R19 PCB landing. A dashed purple internal board jumper continues from that landing to Heltec GPIO6 (physical hole marked “6”, engineering J3.17 at C4,R13). These are one electrical path; the internal jumper is not a second external wire or landing. Its drawn route is provisional insulated underside wiring. Every matching net in the table must be joined using appropriate insulated underside wiring. The preview does not specify a completed jumper routing or wire gauge. Adjacent pads are electrically isolated; line crossings do not create connections.

| Reference | Description | Pin | Column | Row | X mm | Y mm | Net | Status / assembly note |
|---|---|---|---|---|---|---|---|---|
| U3 | Heltec V4 | J2.1 | 20 | 4 | 50.80 | 10.16 | GND | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.2 | 19 | 4 | 48.26 | 10.16 | +5V | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.3 | 18 | 4 | 45.72 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.4 | 17 | 4 | 43.18 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.5 | 16 | 4 | 40.64 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.6 | 15 | 4 | 38.10 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.7 | 14 | 4 | 35.56 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.8 | 13 | 4 | 33.02 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.9 | 12 | 4 | 30.48 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.10 | 11 | 4 | 27.94 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.11 | 10 | 4 | 25.40 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.12 | 9 | 4 | 22.86 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.13 | 8 | 4 | 20.32 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.14 | 7 | 4 | 17.78 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.15 | 6 | 4 | 15.24 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.16 | 5 | 4 | 12.70 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.17 | 4 | 4 | 10.16 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J2.18 | 3 | 4 | 7.62 | 10.16 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.1 | 20 | 13 | 50.80 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.2 | 19 | 13 | 48.26 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.3 | 18 | 13 | 45.72 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.4 | 17 | 13 | 43.18 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.5 | 16 | 13 | 40.64 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.6 | 15 | 13 | 38.10 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.7 | 14 | 13 | 35.56 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.8 | 13 | 13 | 33.02 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.9 | 12 | 13 | 30.48 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.10 | 11 | 13 | 27.94 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.11 | 10 | 13 | 25.40 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.12 | 9 | 13 | 22.86 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.13 | 8 | 13 | 20.32 | 33.02 | ADC_NODE | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.14 | 7 | 13 | 17.78 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.15 | 6 | 13 | 15.24 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.16 | 5 | 13 | 12.70 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.17 | 4 | 13 | 10.16 | 33.02 | WIND_UART_RX_GPIO6 | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U3 | Heltec V4 | J3.18 | 3 | 13 | 7.62 | 33.02 | UNUSED BY PEARL | Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions. |
| U1 | Murata horizontal regulator | 1 | 24 | 11 | 60.96 | 27.94 | PROTECTED_12V | 2.54 mm pin pitch; body-to-pad registration provisional; H version retained. |
| U1 | Murata horizontal regulator | 2 | 25 | 11 | 63.50 | 27.94 | GND | 2.54 mm pin pitch; body-to-pad registration provisional; H version retained. |
| U1 | Murata horizontal regulator | 3 | 26 | 11 | 66.04 | 27.94 | +5V | 2.54 mm pin pitch; body-to-pad registration provisional; H version retained. |
| D1 | 1N5822 | A | 23 | 4 | 58.42 | 10.16 | BATT_RAW_12V | 7.62 mm formed lead span provisional. Proven V1: lightly reduce leads if needed to fit holes; remove debris; do not force. |
| D1 | 1N5822 | K/band | 26 | 4 | 66.04 | 10.16 | PROTECTED_12V | 7.62 mm formed lead span provisional. Proven V1: lightly reduce leads if needed to fit holes; remove debris; do not force. |
| D2 | P6KE16A | K/band | 28 | 3 | 71.12 | 7.62 | PROTECTED_12V | 12.70 mm formed lead span provisional; maximum body 7.6 × Ø3.6 mm. |
| D2 | P6KE16A | A | 28 | 8 | 71.12 | 20.32 | GND | 12.70 mm formed lead span provisional; maximum body 7.6 × Ø3.6 mm. |
| R1 | 4.7k | 1 | 16 | 15 | 40.64 | 38.10 | PROTECTED_12V | Provisional/TBD body and formed lead span 10.16 mm. |
| R1 | 4.7k | 2 | 12 | 15 | 30.48 | 38.10 | DIVIDER_NODE | Provisional/TBD body and formed lead span 10.16 mm. |
| R2 | 1k | 1 | 12 | 16 | 30.48 | 40.64 | DIVIDER_NODE | Provisional/TBD body and formed lead span 10.16 mm. |
| R2 | 1k | 2 | 8 | 16 | 20.32 | 40.64 | GND | Provisional/TBD body and formed lead span 10.16 mm. |
| R3 | 100R | 1 | 12 | 14 | 30.48 | 35.56 | DIVIDER_NODE | Provisional/TBD body and formed lead span 10.16 mm. |
| R3 | 100R | 2 | 8 | 14 | 20.32 | 35.56 | ADC_NODE | Provisional/TBD body and formed lead span 10.16 mm. |
| C1 | 1uF electrolytic | + | 12 | 17 | 30.48 | 43.18 | DIVIDER_NODE | Provisional/TBD body and formed lead spacing 5.08 mm; do not force body leads to assumed pitch. |
| C1 | 1uF electrolytic | - | 10 | 17 | 25.40 | 43.18 | GND | Provisional/TBD body and formed lead spacing 5.08 mm; do not force body leads to assumed pitch. |
| C2 | 0.1uF ceramic | 1 | 8 | 15 | 20.32 | 38.10 | ADC_NODE | Provisional/TBD body and formed lead spacing 5.08 mm. |
| C2 | 0.1uF ceramic | 2 | 6 | 15 | 15.24 | 38.10 | GND | Provisional/TBD body and formed lead spacing 5.08 mm. |
| WIRE | Battery positive (wire 1) | 12V+ | 23 | 1 | 58.42 | 2.54 | BATT_RAW_12V | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |
| WIRE | Battery negative (wire 2) | 12V− | 24 | 1 | 60.96 | 2.54 | GND | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |
| WIRE | WindSonic pin 2 supply+ (wire 3) | PROT+ | 25 | 1 | 63.50 | 2.54 | PROTECTED_12V | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |
| WIRE | Waveshare TXD → Heltec GPIO6 (pin marked “6” on board), purple jumper (wire 4) | TXD | 5 | 19 | 12.70 | 48.26 | WIND_UART_RX_GPIO6 | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |
| WIRE | Waveshare TTL-side GND (wire 5) | GND | 7 | 19 | 17.78 | 48.26 | GND | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |
| WIRE | Waveshare VCC +5V (wire 6) | VCC | 11 | 19 | 27.94 | 48.26 | +5V | Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified. |

Manufacturer sources:

- [SchmalzTech mechanical drawing](https://www.schmalztech.com/cdn/shop/files/ST-PERF-2-3_drawing.PDF?v=11111408847333900156)
- [Heltec V4 mechanical documentation](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/datasheet/WiFi_LoRa_32_V4.2.0.pdf)
- [Heltec official pinmap](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/Pinmap/V4_pinmap.png)
- [Murata mechanical drawing](https://www.murata.com/-/media/webrenewal/products/power/datasheet/oki-78sr.ashx?cvid=20200224052008000000&la=en-us)
- [Microchip diode drawing](https://ww1.microchip.com/downloads/aemDocuments/documents/HRDS/ProductDocuments/DataSheets/1N5822-DSB5820-5822.pdf)
- [Littelfuse P6KE drawing](https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_p6ke_datasheet.pdf.pdf)
