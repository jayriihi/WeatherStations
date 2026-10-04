from pathlib import Path
import csv, html
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).parent
W,H=1900,1900
im=Image.new('RGB',(W,H),'#f6f8fb'); d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', '<rect width="100%" height="100%" fill="#f6f8fb"/>']
fontpath='/System/Library/Fonts/Supplemental/Arial.ttf'
def text(x,y,s,size=20,color='#253449'):
    d.text((x,y),s,font=ImageFont.truetype(fontpath,size),fill=color)
    svg.append(f'<text x="{x}" y="{y+size*.85}" font-family="Arial,sans-serif" font-size="{size}" fill="{color}">{html.escape(s)}</text>')
def line(points,color='#53657a',width=2):
    d.line(points,fill=color,width=width)
    svg.append(f'<polyline points="{" ".join(f"{x},{y}" for x,y in points)}" fill="none" stroke="{color}" stroke-width="{width}"/>')
def rect(box,fill,stroke='#53657a',width=2):
    d.rectangle(box,fill=fill,outline=stroke,width=width)
    x,y,x2,y2=box;svg.append(f'<rect x="{x}" y="{y}" width="{x2-x}" height="{y2-y}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
def circle(x,y,r,fill,stroke='#53657a',width=2):
    d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=stroke,width=width)
    svg.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
S=13;OX,OY=155,300
def pt(x,y):return OX+x*S,OY+y*S
def box(x,y,w,h,fill,stroke='#53657a'):rect((*pt(x,y),*pt(x+w,y+h)),fill,stroke)
def pad(c,r,color='#fff',radius=6):circle(*pt(c*2.54,r*2.54),radius,color)
def label(x,y,s,size=18,color='#253449'):text(*pt(x,y),s,size,color)

text(70,40,'PEARL V2  |  PROVISIONAL PHYSICAL ASSEMBLY',34)
text(70,88,'Component side • ST-PERF-2-3 • millimetres • review placement before soldering',21)
text(70,123,'Electrical design unchanged. Passive bodies, headers and wire routes remain provisional.',19,'#a45a10')
box(0,0,76.2,50.8,'#eef0df','#253449')
for c in range(1,30):
    for r in range(1,20):
        if (r in (1,19) and c in (1,2,28,29)) or (r in (2,18) and c in (1,29)):continue
        pad(c,r,'#fbfcf5',5)
for c in range(1,30):text(pt(c*2.54,0)[0]-7,OY-30,str(c),14)
for r in range(1,20):text(OX-35,pt(0,r*2.54)[1]-9,str(r),14)
for x,y in [(2.54,2.54),(73.66,2.54),(2.54,48.26),(73.66,48.26)]:
    circle(*pt(x,y),4.5*S,'#fde9d3','#d99548')
    circle(*pt(x,y),1.5875*S,'#f6f8fb','#253449')
label(4,49.2,'76.20 × 50.80 mm / 2.54 mm grid',17)

# Manufacturer-sized module envelope; grid anchoring remains provisional.
box(3.81,8.89,51.69,25.4,'#dce8f2','#36708d')
box(12,14,32,14,'#31465d','#31465d')
label(15,18,'HELTEC V4',25,'#ffffff')
label(14,23,'Display / component side up',16,'#ffffff')
for n in range(1,19):
    c=21-n
    pad(c,4,'#d6e5f4',7);pad(c,13,'#d6e5f4',7)
label(6,11.2,'J2  ← 18 … 1 →',17)
label(6,30.4,'J3  ← 18 … 1 →',17)
for c,r,t,col in [(20,4,'GND','#35485b'),(19,4,'5V','#cc4c42'),(8,13,'GPIO2','#25886d'),(4,13,'GPIO6','#8456b0')]:
    pad(c,r,col,8)
label(15,32.2,'ADC',14,'#25886d');label(6.7,32.2,'6',14,'#8456b0')
box(53,17.3,2.5,8,'#a6b6c7','#36708d')
box(56.5,18.5,19.7,8,'#fff3dc','#d99548')
label(57,20,'USB access',17,'#9a5b17');label(57,23,'keep clear',16,'#9a5b17')
label(5,6.8,'Header/socket body & vertical clearance TBD',15,'#36708d')

# Purple path legend: no second external callout at GPIO6.
text(170,210,'Dashed purple: INTERNAL board jumper',17,'#8456b0')
text(170,235,'Solid purple: ONE external TXD wire',17,'#8456b0')

# Murata horizontal module; body-to-pin offset is provisional.
box(58.3,29.21,10.4,16.5,'#d8ebdf','#347450')
label(59,34,'U1',23,'#347450');label(59,37,'Murata',17,'#347450')
label(59,40,'H mount',16,'#347450');label(59,43,'10.4 × 16.5',13,'#347450')
for c,n in [(24,'1'),(25,'2'),(26,'3')]:
    pad(c,11,'#b8d5c4',8);label(c*2.54-.5,25.8,n,15)
label(58,47,'Pin/body offset provisional',13,'#347450')

def axial(ref,a,b,color,body=None):
    pa,pb=pt(a[0]*2.54,a[1]*2.54),pt(b[0]*2.54,b[1]*2.54)
    line([pa,pb],color,4);pad(*a,color,7);pad(*b,color,7)
    if body:
        x,y,w,h=body;box(x,y,w,h,color,color)
    else:
        mx=(pa[0]+pb[0])/2;my=(pa[1]+pb[1])/2
        rect((mx-22,my-10,mx+22,my+10),'#fff1cf',color)
    text((pa[0]+pb[0])/2-16,(pa[1]+pb[1])/2-30,ref,18,color)
axial('D1',(23,4),(26,4),'#ac5e27',(60.5,8.4,3.7,3.5))
label(64.6,11.2,'K →',14,'#ac5e27')
axial('D2',(28,3),(28,8),'#ac5e27',(69.32,10.16,3.6,7.6))
label(60,6.2,'D1',18,'#ac5e27');label(67,8,'D2',18,'#ac5e27')
label(72.2,8,'K',14,'#ac5e27');label(72.2,19,'A',14,'#ac5e27')
label(58,14,'Protection / power entry',14,'#ac5e27')

passives=[('R1',(16,15),(12,15)),('R2',(12,16),(8,16)),('R3',(12,14),(8,14)),('C1',(12,17),(10,17)),('C2',(8,15),(6,15))]
for ref,a,b in passives:axial(ref,a,b,'#986a11')
label(32,42,'ADC zone',18,'#986a11');label(32,45,'R/C bodies & pitches TBD',14,'#986a11')
line([pt(20.32,33.02),pt(20.32,35.56)],'#25886d',3)

# External solder pads, all coordinates real existing holes.
edge=[('12V+',23,1,'#cf5145'),('12V−',24,1,'#35485b'),('PROT+',25,1,'#cf5145'),('TXD',5,19,'#8456b0'),('GND',7,19,'#35485b'),('VCC',11,19,'#cf5145')]
for name,c,r,col in edge:pad(c,r,col,9)
full_labels=['1. BATTERY 12V+','2. BATTERY 12V−','3. WINDSONIC PROTECTED 12V+','4. TXD: Waveshare TXD → Heltec GPIO6 (pin marked “6” on board)','5. WAVESHARE GND','6. WAVESHARE VCC (+5V)']
for i,(name,c,r,col) in enumerate(edge[:3]):
    x,y=pt(c*2.54,r*2.54);ty=165+i*40
    text(600,ty,full_labels[i]+f'  [C{c}, R{r}]',19,col)
    line([(x,y),(x,ty+28),(600,ty+28)],col,2)
# Separate full landing labels with leaders immediately below the board edge.
for idx,tx,ty in [(3,65,1010),(4,395,1055),(5,555,990)]:
    name,c,r,col=edge[idx];px,py=pt(c*2.54,r*2.54)
    caption=full_labels[idx]+f'  [C{c}, R{r}]'
    if idx==3:
        text(tx,ty,'4. TXD — ONE WIRE',18,col)
        text(tx,ty+24,'Landing C5, R19 → GPIO6',18,col)
        text(tx,ty+48,'(physical hole marked “6”)',18,col)
    else:text(tx,ty,caption,18,col)
    if idx!=3:line([(px,py),(px,ty-8),(tx,ty-8)],col,2)
line([pt(6,22),pt(-7,22)],'#3877a6',4)
text(20,OY+22*S-40,'LoRa coax',17,'#3877a6');text(20,OY+22*S-17,'to SMA / antenna',15,'#3877a6')

# One internal jumper joins the sole TXD landing to GPIO6. All points stay on board.
import math
internal=[pt(12.70,48.26),pt(12.70,35.56),pt(10.16,35.56),pt(10.16,33.02)]
for a,b in zip(internal,internal[1:]):
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
    for start in range(0,int(length)+1,12):
        end=min(start+7,length)
        if start<length:line([(a[0]+dx*start/length,a[1]+dy*start/length),(a[0]+dx*end/length,a[1]+dy*end/length)],'#8456b0',3)
pad(5,19,'#8456b0',9);pad(4,13,'#8456b0',8)
label(1,35,'INTERNAL',13,'#8456b0');label(1,37,'jumper',14,'#8456b0')
text(170,180,'GPIO6 = physical hole marked “6”',16,'#8456b0')

rect((170,1250,1155,1450),'#e6eaf0','#53657a')
text(745,1265,'WAVESHARE SKU 23652',22)
text(745,1297,'External DIN rail • logical view • not to scale',16)
text(195,1265,'3 PHYSICAL TTL WIRES',17)
termx=[320,440,680]
for i,(name,c,r,col) in enumerate(edge[3:]):
    px,py=pt(c*2.54,r*2.54);tx=termx[i]
    line([(px,py),(px,1160+i*10),(tx,1160+i*10),(tx,1340)],col,3)
    terminal={'TXD':'TXD','GND':'GND','VCC':'VCC'}[name]
    circle(tx,1340,9,col);text(tx-25,1357,terminal,17,col)
text(195,1410,'TXD USED • Purple jumper: Waveshare TXD → Heltec GPIO6 (pin marked “6” on board)',17)
for i,(term,source) in enumerate([('R+','WindSonic pin 4 / TXD+'),('R−','WindSonic pin 5 / TXD−'),('RGND','WindSonic pin 1 / signal ground')]):
    y=1330+i*32;circle(780,y,7,'#3877a6');text(800,y-12,term,16);line([(850,y),(900,y)],'#3877a6',2);text(915,y-12,source,15)
# Mask wire strokes behind landing-label text for legibility.
for idx,tx,ty in [(3,65,1010),(4,395,1055),(5,555,990)]:
    name,c,r,col=edge[idx];caption=full_labels[idx]+f'  [C{c}, R{r}]'
    if idx==3:
        rect((tx-2,ty-1,tx+242,ty+72),'#f6f8fb','#f6f8fb',1)
        text(tx,ty,'4. TXD — ONE WIRE',18,col)
        text(tx,ty+24,'Landing C5, R19 → GPIO6',18,col)
        text(tx,ty+48,'(physical hole marked “6”)',18,col)
    else:
        tw=d.textlength(caption,font=ImageFont.truetype(fontpath,18))
        rect((tx-2,ty-1,tx+tw+3,ty+23),'#f6f8fb','#f6f8fb',1)
        text(tx,ty,caption,18,col)

rect((1200,170,1845,530),'#e5eff8','#36708d')
text(1220,190,'EXTERNAL WIRES — 6',30,'#253449')
text(1220,240,'PHYSICAL WIRE / BOARD LANDING',18)
for i,(name,c,r,col) in enumerate(edge):
    if i==3:
        text(1220,278+i*40,'4. TXD: Waveshare TXD → Heltec GPIO6',18,col)
        text(1220,300+i*40,'(pin marked “6” on board)',16,col)
    else:text(1220,278+i*40,full_labels[i],18,col)
    text(1740,278+i*40,f'C{c}, R{r}',18,col)
text(1220,560,'PLACEMENT & ASSEMBLY NOTES',24)
notes=[
 'Coordinates: column,row; x = col × 2.54 mm.',
 'Origin: upper-left board corner; y increases down.',
 'Orange circles: Ø9 mm proposed hardware keepouts.',
 'Actual mounting holes: Ø3.175 mm.',
 'Keepouts are design allowances, not vendor dimensions.',
 '',
 'Heltec: USB right; J2 top / J3 bottom.',
 'Pin 1 at right, pin 18 at left on both headers.',
 'Body registration and socket height: provisional.',
 'Keep space under Heltec for insulated wiring.',
 '',
 'U1: horizontal OKI-78SR-5/1.5-W36H-C.',
 'Pins 1 / 2 / 3 = input / GND / +5 V.',
 'Body outline nominal; allow manufacturing tolerance.',
 '',
 'D1: retain 1N5822; cathode toward protected 12 V.',
 'Proven V1 step: lightly reduce leads if needed',
 'to fit holes; clear debris and do not force insertion.',
 'D1 body depiction is schematic, not exact outline.',
 'D2: P6KE16A; cathode protected 12 V, anode GND.',
 '',
 'R1 4.7k / R2 1k / R3 100R:',
 'proposed axial holes; exact packages TBD.',
 'C1 1µF: positive at divider, negative at GND.',
 'C2 0.1µF: ADC node to GND; exact package TBD.',
 'R/C rectangles are markers, not measured bodies.',
 '',
 'Colored edge wires show destinations only.',
 'Dashed purple: landing → GPIO6 internal jumper.',
 'Solid purple: one external TXD wire to Waveshare.',
 'Other underboard jumpers remain unrouted.',
 'Sensor supply+: protected 12 V from Pearl.',
 'Sensor supply−: directly to battery −, off board.',
 'Sensor signal ground stays on Waveshare RGND.',
 'RF coax connects directly to Heltec, not a solder pad.',
 'Waveshare RXD: UNUSED.',
 'Exactly 6 wire landings: 2 in / 1 sensor / 3 TTL.',
 'Purple jumper: Waveshare TXD → Heltec GPIO6',
 '                         (pin marked “6” on board)',
 '',
 'This is a review preview, not a build-ready release.'
]
for i,n in enumerate(notes):text(1220,605+i*28,n,17)
rect((170,1530,1155,1695),'#eef2f5','#53657a')
text(190,1548,'OFF-BOARD WIRING',23)
text(190,1588,'WindSonic supply− → Battery− directly; NOT connected to Pearl PCB',20)
text(190,1623,'Waveshare: DIN-rail mounted externally',20)
text(190,1658,'WindSonic RS-422: WindSonic ↔ Waveshare directly; no RS-422 wires land on Pearl PCB',19)
text(70,1750,'Provisional 08 • 04 October 2026 • Six physical wire landings • electrical design unchanged',20)
text(70,1790,'Grid holes omitted around the four mounting holes reproduce the manufacturer drawing.',18)
text(70,1822,'Internal net membership is specified in the accompanying table; crossing preview lines do not imply connections.',18)
svg.append('</svg>');(OUT/'pearl_provisional_layout.svg').write_text('\n'.join(svg))
im.save(OUT/'pearl_provisional_layout.png')

rows=[]
def add(ref,desc,pins,note):
    for pin,c,r,net in pins:rows.append([ref,desc,pin,c,r,f'{c*2.54:.2f}',f'{r*2.54:.2f}',net,note])
add('U3','Heltec V4',[(f'J2.{n}',21-n,4,{1:'GND',2:'+5V'}.get(n,'UNUSED BY PEARL')) for n in range(1,19)]+[(f'J3.{n}',21-n,13,{13:'ADC_NODE',17:'WIND_UART_RX_GPIO6'}.get(n,'UNUSED BY PEARL')) for n in range(1,19)],'Provisional grid anchoring; USB right. Unused means no proposed Pearl jumper, not removal of onboard functions.')
add('U1','Murata horizontal regulator',[('1',24,11,'PROTECTED_12V'),('2',25,11,'GND'),('3',26,11,'+5V')],'2.54 mm pin pitch; body-to-pad registration provisional; H version retained.')
add('D1','1N5822',[('A',23,4,'BATT_RAW_12V'),('K/band',26,4,'PROTECTED_12V')],'7.62 mm formed lead span provisional. Proven V1: lightly reduce leads if needed to fit holes; remove debris; do not force.')
add('D2','P6KE16A',[('K/band',28,3,'PROTECTED_12V'),('A',28,8,'GND')],'12.70 mm formed lead span provisional; maximum body 7.6 × Ø3.6 mm.')
add('R1','4.7k',[('1',16,15,'PROTECTED_12V'),('2',12,15,'DIVIDER_NODE')],'Provisional/TBD body and formed lead span 10.16 mm.')
add('R2','1k',[('1',12,16,'DIVIDER_NODE'),('2',8,16,'GND')],'Provisional/TBD body and formed lead span 10.16 mm.')
add('R3','100R',[('1',12,14,'DIVIDER_NODE'),('2',8,14,'ADC_NODE')],'Provisional/TBD body and formed lead span 10.16 mm.')
add('C1','1uF electrolytic',[('+',12,17,'DIVIDER_NODE'),('-',10,17,'GND')],'Provisional/TBD body and formed lead spacing 5.08 mm; do not force body leads to assumed pitch.')
add('C2','0.1uF ceramic',[('1',8,15,'ADC_NODE'),('2',6,15,'GND')],'Provisional/TBD body and formed lead spacing 5.08 mm.')
edge_nets=['BATT_RAW_12V','GND','PROTECTED_12V','WIND_UART_RX_GPIO6','GND','+5V']
edge_dest=['Battery positive (wire 1)','Battery negative (wire 2)','WindSonic pin 2 supply+ (wire 3)','Waveshare TXD → Heltec GPIO6 (pin marked “6” on board), purple jumper (wire 4)','Waveshare TTL-side GND (wire 5)','Waveshare VCC +5V (wire 6)']
for (name,c,r,col),net,dest in zip(edge,edge_nets,edge_dest):add('WIRE',dest,[(name,c,r,net)],'Proposed soldered wire landing; provide enclosure strain relief. Connector body not specified.')
head=['Reference','Description','Pin','Column','Row','X mm','Y mm','Net','Status / assembly note']
with (OUT/'hole_coordinates.csv').open('w',newline='') as f:
    cw=csv.writer(f);cw.writerow(head);cw.writerows(rows)
md=['# Pearl V2 provisional component-side placement','',
 'Review preview only. Electrical design unchanged. Origin is the upper-left corner of the component-side view; columns increase right and rows increase down. Hole (C,R) is at (2.54C, 2.54R) mm. No files outside this preview directory were changed by the drawing script.','',
 'Board: ST-PERF-2-3, 76.20 × 50.80 mm; 0.040-inch component holes. Mounting centres: (2.54,2.54), (73.66,2.54), (2.54,48.26), (73.66,48.26) mm; diameter 3.175 mm. The orange Ø9 mm hardware clearances are provisional design allowances. Corner omissions follow the manufacturer drawing.','',
 'Heltec: USB right, display upward, J2 upper and J3 lower, pin 1 at right. Header pitch is 2.54 mm and row separation 22.86 mm. Drawn body 51.69 × 25.40 mm at x=3.81, y=8.89 mm; longitudinal envelope registration remains provisional pending complete mechanical callout review. Socket bodies, mounting height and underside clearance are TBD. This drawing does not assign extra Heltec mounting screws.','',
 'Murata: exact horizontal H model retained. Drawn 10.4 × 16.5 mm nominal body; body-to-pin offset provisional. Manufacturer dimensional tolerances and physical assembly clearances must be checked before build release.','',
 'D1 is the retained Microchip 1N5822, not a substituted generic diode. Its body marker is illustrative; the formed lead span is provisional. Proven Pearl V1 procedure: lightly sand/reduce leads only if necessary to fit the perfboard holes, remove sanding debris, and insert without force. Cathode/band faces PROTECTED_12V.','',
 'R1–R3 and C1–C2: proposed hole locations only. Exact manufacturer packages, body dimensions and natural lead pitches remain TBD; the drawn markers are not component-size claims. C1 positive is DIVIDER_NODE.','',
 'Authoritative physical arrangement supplied by the user, 04 October 2026: exactly SIX separate board wire landings. Wires 1–2: battery 12V+ and 12V− into Pearl. Wire 3: protected 12V+ to WindSonic. WindSonic supply− / J2.3 connects DIRECTLY to battery negative off the Pearl board; it has no Pearl wire landing. Wires 4–6: TXD (user harness label), GND and VCC (+5V) to Waveshare. Battery return and Waveshare return are separate physical PCB wires despite sharing GND. Former C26,R1 sensor-return and C9,R19 extra-module-ground landings are unassigned. This preserves the electrical GND membership in the engineering schematic.','',
 'WORKING HARDWARE AUTHORITY: Waveshare TXD is USED and sends data over the purple TXD wire to Heltec GPIO6, at the physical underside header hole marked “6”. Waveshare RXD is UNUSED. The assembler-facing connection is Waveshare TXD → TXD wire → Heltec GPIO6 (pin marked “6” on board). The existing board net name WIND_UART_RX_GPIO6 and engineering cross-reference J3.17 are retained; receive refers to the Heltec role. The physical terminal identification is supplied by the user. No engineering schematic, firmware or PCB edits are made.', '',
 'External connections: battery ± and sensor supply+ leave the upper edge. Sensor supply− goes directly to battery negative, not through a Pearl landing. Waveshare TXD harness wire, GND and +5V VCC leave the lower edge in left-to-right order TXD → GND → VCC: TXD at C5,R19 (12.70,48.26 mm), GND at C7,R19 (17.78,48.26 mm), VCC at C11,R19 (27.94,48.26 mm). Only TXD and VCC wire landings were exchanged; all component positions and electrical nets are unchanged. The schedule’s U2.GND1/GND2 describe electrical ground membership, not two board-to-module conductors; one physical ground lead leaves Pearl as instructed. WindSonic pin 4 → Waveshare R+, pin 5 → R−, pin 1 → RGND; these bypass the perfboard. Unused Waveshare RXD, T+, T− and sensor pins 6–9 remain unwired. Signal ground is distinct from logic/battery ground. Heltec LoRa IPEX → SMA bulkhead pigtail → antenna leaves toward the left; no perfboard RF pad is added. The six-wire count excludes that coax and temporary USB. Waveshare remains external on its DIN rail.','',
 'The sole TXD external wire runs from Waveshare TXD to the C5,R19 PCB landing. A dashed purple internal board jumper continues from that landing to Heltec GPIO6 (physical hole marked “6”, engineering J3.17 at C4,R13). These are one electrical path; the internal jumper is not a second external wire or landing. Its drawn route is provisional insulated underside wiring. Every matching net in the table must be joined using appropriate insulated underside wiring. The preview does not specify a completed jumper routing or wire gauge. Adjacent pads are electrically isolated; line crossings do not create connections.','',
 '| '+' | '.join(head)+' |','|'+'|'.join(['---']*len(head))+'|']
md += ['| '+' | '.join(map(str,row))+' |' for row in rows]
md += ['', 'Manufacturer sources:', '',
 '- [SchmalzTech mechanical drawing](https://www.schmalztech.com/cdn/shop/files/ST-PERF-2-3_drawing.PDF?v=11111408847333900156)',
 '- [Heltec V4 mechanical documentation](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/datasheet/WiFi_LoRa_32_V4.2.0.pdf)',
 '- [Heltec official pinmap](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/Pinmap/V4_pinmap.png)',
 '- [Murata mechanical drawing](https://www.murata.com/-/media/webrenewal/products/power/datasheet/oki-78sr.ashx?cvid=20200224052008000000&la=en-us)',
 '- [Microchip diode drawing](https://ww1.microchip.com/downloads/aemDocuments/documents/HRDS/ProductDocuments/DataSheets/1N5822-DSB5820-5822.pdf)',
 '- [Littelfuse P6KE drawing](https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_p6ke_datasheet.pdf.pdf)']
(OUT/'hole_coordinates.md').write_text('\n'.join(md)+'\n')
print('Created provisional SVG, PNG, CSV and Markdown table.')
