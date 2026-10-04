# Pearl V2 Assembly & Wiring

Open `pearl_v2_assembly_wiring.kicad_sch` for the visual overview, and
`pearl_v2_assembly_connections.kicad_sch` for the exact connection schedule.
These are editable KiCad 10 graphical documents, not replacement engineering
schematics and not PCB placement drawings. The two standalone sheets use
same-row terminal lists and annotated flow paths instead of electrical symbols.

Electrical authority: `../pearl-v2-schematic/pearl_v2_schematic/pearl_v2_schematic.kicad_sch`.
Component references, values and net membership were read from that schematic's
KiCad-exported netlist. The original documents, PCB and firmware are untouched.
The older legacy README/BOM contain superseded GPIO7/GPIO19 references; the
current engineering schematic establishes GPIO6 / Heltec J3.17.

Physical pad locations, component placement, Heltec mounting orientation,
connector viewing direction and converter mounting location are not established.
This drawing cannot resolve those details without additional physical evidence.
Do not interpret the diagram's positions as mounting instructions. All documented
unresolved specifications remain unresolved; no circuit changes are proposed.

Drawn lines show the stated logical routes. Full net membership is listed on
sheet 2; matching named nets are the same electrical connection. Signal ground
is kept distinct from logic/battery ground, as in the source schematic.

Both sheets have their own `.kicad_pro` and share `assembly_page.kicad_wks`,
a minimal page border without the engineering title block. Keep these files
together.

## Working hardware terminal identification — 04 October 2026

The user-confirmed physical authority is **Waveshare TXD → purple TXD wire → Heltec GPIO6 (pin marked “6” on board)**. TXD is USED; Waveshare RXD is UNUSED. All generated assembly annotations and previews use this physical terminal identification. The engineering schematic is unchanged; its earlier module-terminal naming is not the physical assembly authority for this correction. Component placement and six external board wire landings are unchanged.
