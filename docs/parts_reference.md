# Parts reference: every part in the thesis, with links and CAD

Compiled 2026-11-11 from the bachelor thesis (`../../Bachelorarbeit_Leonard_Zensen_1211472_WS2526 (1).pdf`):
the text, the photos, Anhang A–D, and the Literaturverzeichnis. Shop and datasheet links were opened on
2026-11-11 unless marked otherwise. Downloaded CAD and datasheets are in `../../cad/` (one folder per part,
each with a `SOURCE.txt` giving origin, licence and measured size). Facts already in `thesis_notes.md` are not
repeated here.

Legend: **local** = file in `../../cad/`; **online** = free download you have to fetch yourself (login or bot
block noted); **none** = no CAD exists that I could find, so measure the part.

## 1. Discrepancies to resolve first

| # | What | Thesis says | Evidence against | Action |
|---|---|---|---|---|
| 1 | Linear rail size and length | Text §4.5: "MGN12 … 700mm". BOM note: "ca. 60-70cm, MG12 verbaut" | BOM **link** is `linearfuehrung-mgn15h-600mm` (MGN15H, **600 mm**, roboter-bausatz RBS12934). Simon measured MGN15 hole spacing on the rig (2026-11-03). | Measure the rail length. `ml-first-principles/config/pendel.yaml` `track.rail_length` = 0.70 m cites "thesis BOM: MGN12", which is wrong on the size and possibly on the length. |
| 2 | Carriage | BOM row "Linearführungswagen" links an **MGN12C** carriage, note: "bereits bei aktueller Schiene enthalten!" | The note says the rail came with its own carriage, so the MGN12C was probably never fitted. On an MGN15 rail it cannot be. | The carriage is the MGN15H that came with the rail. Check the 4 × M3 holes are on 25 × 25 mm. |
| 3 | Encoder part number | "B38S6G5-600B-G24N" | Every listing found uses **E38S6G5-600B-G24N**. The thesis photo (Abb. 5) cuts off the first letter. | Read the label. Treat the specs as E38S6G5. |
| 4 | Bulk capacitor | BOM: "100µF Kondensator" | Schematic (Anhang B): 300 µF | Check the part on the board. |
| 5 | Pull-ups | 4 × 8 kΩ | The new PCB uses 4.7 kΩ (encoder) and 10 kΩ (switches), per `CLAUDE.md` | Only matters when reading the thesis schematic. |
| 6 | Pi RAM | BOM link: 4 GB | The original rig Pi was 1 GB (`stage0_discovery.md`). The replacement since 2026-10-23 is 4 GB. | None. |

## 2. Links in the thesis (complete)

Literaturverzeichnis and body text (embedded hyperlinks):
- Motor datasheet, ACT 23HS2430: https://cdn-reichelt.de/documents/datenblatt/C300/23HS2430.pdf
- Raspberry Pi 4 (cited as "Datenblatt", actually the shop page): https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_4_b_4x_1_8_ghz_4_gb_ram_wlan_bt-259920?PROVID=2788
- Schmitt trigger CD4093B-Q1 (filter board, later removed): https://www.ti.com/lit/ds/symlink/cd4093b-q1.pdf
- Quadrature decoding reference, Paul Stoffregen's Encoder library: https://github.com/PaulStoffregen/Encoder (and `/blob/master/Encoder.h`)
- Code: https://github.com/lzprograms/Pendel (code only, **no CAD or STL files**; checked 2026-11-11). The author's
  Windows client is a separate repo: https://github.com/lzprograms/Pendelclient
- Test video (HSBI Ilias, needs a university login): https://www.hsbi.de/elearning/goto.php/fold/1664517
- A4988 manual from AZ-Delivery: emailed at purchase, no URL. Anhang A reproduces its pages.
- Diagram tool: https://mermaid.live

BOM (Anhang D) "Link" column, transcribed from the embedded image:

| BOM row | Link | Status 2026-11-11 |
|---|---|---|
| Netzteil 24V | https://www.leds24.com/Netzteil-stabilisiert-24VDC-5A-120Watt | live: SANPU "Power Desk 120W", 24 V 5 A, 5.5 × 2.5 mm plug, 56 × 40 × 134 mm |
| Schrittmotor | https://www.reichelt.de/de/de/shop/produkt/schrittmotor_4_pol_1_8_4_8_v_dc-229320?PROVID=2788 | live: ACT 23HS2430 |
| Raspberry Pi 4B | https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_4_b_4x_1_8_ghz_4_gb_ram_wlan_bt-259920?PROVID=2788 | link from the Literaturverzeichnis (the BOM cell is cut off) |
| Riemen | https://www.roboter-bausatz.de/p/10-meter-gt2-zahnriemen-offen-6mm | live: RBS10449, GT2 6 mm, rubber with fibreglass cords, 10 m roll |
| Schrittmotortreiber | https://www.roboter-bausatz.de/p/a4988-schrittmotorentreiber | live: RBS10232, MakerMind A4988 carrier with heatsink |
| Linearführungsschiene | https://www.roboter-bausatz.de/p/linearfuehrung-mgn15h-600mm | live: RBS12934, MGN15H 600 mm, 655 g, non-HIWIN clone ("your droid") |
| Linearführungswagen | https://3d-druckershop.com/p/mgn12c-linearfuehrungswagen-kaufen | not read: the site's TLS certificate does not match its hostname |
| Umlenkrolle Riemen | https://3d-druckershop.com/p/fag-miniatur-rillenkugellager-608-zz-2z-8x22x7-mm-kaufen | not read (same certificate problem). FAG 608-2Z, 8 × 22 × 7 |
| Wippschalter | https://www.reichelt.de/de/de/shop/produkt/wippenschalter_rund_1x_ein_-_aus-105442 | live: SCI R13-112, Ø23, cutout 20.2 mm |
| Sicherungskasten | https://www.reichelt.de/de/de/shop/produkt/sicherungshalter_80_v_15_a-242449 | live: Littelfuse 03500417TXN, 5-way ATO block |
| Touchscreen | https://www.reichelt.com/ch/de/shop/produkt/raspberry_-_display_lcd-touch_7_ips-317040 | live: Joy-IT RB-LCD-7-3, 1024 × 600, 165 × 124 × 13 mm |
| Kabel/Drähte | `https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_-_steckbrueckenkabel_25cm_40-polig_auf_dupon-17…` | **cut off in the BOM image**: 25 cm 40-way ribbon to Dupont, article number incomplete |
| Spannungswandler | https://www.roboter-bausatz.de/p/spannungswandler-xy-3606-dc-dc-stepdown-modul-9-36v-auf-5v | live: RBS15567, XY-3606, 9–36 V to 5.2 V, 5 A at 24 V, 36 × 64 mm |

The BOM's priced rows sum to 294.84 €. Rows without a price (pulley, board, screws, printed parts, etc.) are not
included.

## 3. Mechanical parts

| Part | Thesis / BOM | What the photos show | Datasheet / shop | CAD |
|---|---|---|---|---|
| **Stepper motor** ACT 23HS2430 | §4.1, BOM "Schrittmotor" 28.30 € | NEMA 23 on a galvanised angle bracket, rubber mat underneath (Abb. 2/3) | ACT drawing: `cad/motor_23HS2430/23HS2430_reichelt_X200_DATASHEET.pdf`. 56.4 square, 112 mm body, 47.14 bolt square, 4 × Ø5, pilot Ø38.1 × 1.6, **round Ø8 × 21 shaft**, 280 N·cm, 800 g·cm² rotor | **local**: `23HS2430.step` generated from that drawing by `make_23hs2430.py` (checked bbox 56.4 × 56.4 body, 133 overall). ACT publishes no CAD |
| **Linear rail** MGN15 (see discrepancy 1) | §4.5, BOM 30.80 € | Rail screwed to the board, one carriage (Abb. 10/11) | Shop info sheet + HIWIN MGN/MGW catalogue 2023 in `cad/MGN15/` | **local**: `HIWIN_MGNR15_rail_L1000_STEP214.stp` (official HIWIN via Damen CNC, cut to length), `MGN15-Rail-700.step` (generated, use `--length` to remake), cross-section DXF |
| **Carriage** MGN15H | BOM 10.15 € (link is MGN12C) | 3D-printed cart bolted on a green-sealed block | HIWIN catalogue (local) | **local**: `HIWIN_MGN15H_carriage_STEP214.stp` (official, 32 × 63.7 × 12). MGN12C/H also in `cad/MGN12/` in case |
| **Timing belt** GT2, 6 mm | §4.6, BOM 11.95 €, "≥1500 mm" | Open belt, both ends clamped on the cart | roboter-bausatz RBS10449 (§2). Profile: Gates 2MGT/GT2, 2 mm pitch | **none** needed. Model as a 6 × 1.38 mm band, pitch line 0.254 mm below the tooth tips |
| **Motor pulley** GT2 60T, 8 mm bore | §4.6, BOM "Riemenscheibe" (no link or price) | Aluminium pulley on the motor shaft (Abb. 3, too blurred for flange or screw details) | Example vendor data (one seller): flange Ø44, height 16, ±5 %. Tooth geometry: pitch Ø 38.20, tip Ø 37.69 (from 60 × 2 / π) | **none** official. Measure flange Ø, hub Ø and length, then model it. A parametric GT2 OpenSCAD module exists if you want teeth |
| **Pendulum hub** GT2 pulley on the encoder shaft | §4.2: the pendulum rod screws in "as a replacement for one of the grub screws" | Small aluminium flanged pulley (~16–20T), bore must be 6 mm (Abb. 6) | none | **none**. Count the teeth and measure. The M3 thread for the rod is the grub-screw hole |
| **Idler bearing** 608-2Z (8 × 22 × 7) | §4.6, BOM "Umlenkrolle" 3.15 € | Bearing between washers in a printed fork, on a screw axle (Abb. 13) | Schaeffler medias page (online; CAD after free sign-in): https://medias-at.schaeffler.com/en/product/rotary/rolling-and-plain-bearings/ball-bearings/deep-groove-ball-bearings/608-2z-hln/p/353531 | **local**: `cad/bearing_608ZZ/608ZZ_Ball_Bearing.step` (FreeCAD-library, 22 × 7 checked) |
| **Belt tensioner** (printed, 2 parts) | §4.6: bearing block screwed to the board, distance set by a nut | Printed block + roller fork; a **plastic wing knob** sets the tension (Abb. 13) | none | **none**. Printed-part files were never published (GitHub repo has code only). Ask the author or measure |
| **Cart / encoder holder** (printed) | §4.5: holds the encoder, belt clamp, cable guide | Black printed box on the carriage, cable gland on top, belt ends clamped at both sides (Abb. 10/11) | none | **none**, same as above |
| **Limit-switch blocks** (printed) | BOM "Böcke für Endschalter" | Printed bracket, switch screwed on with 2 screws (Abb. 8) | none | **none** |
| **Limit switches** Heschen V-156-1C25 ×2 | §4.3, Anhang B (NO to GND) | Orange Heschen V-156-1C25 clone, label readable in Abb. 8 | Omron V-series datasheet `cad/limit_switch_V-156-1C25/` (body 27.8 × 15.9 × 10.3 mm, 2 × Ø3.1 holes 22.2 apart; code 6 = hinge roller lever) | **online**: official Omron STEP/IGES/DXF, free, https://download.ia.omron.com/download/page/V_156_1C25/ECB (blocked automated download, so open it in a browser) |
| **Encoder** E38S6G5-600B-G24N | §4.2: 600 P/R, Ø6 shaft, Ø40 housing, 3 tapped holes at 120°, NPN open collector, 5–24 V | Black can with aluminium flange and boss (Abb. 5) | No manufacturer datasheet found (unbranded). Wiring notes: https://forum.arduino.cc/t/fyi-interfacing-e38s6g5-600b-g24n-600p-r-rotary-encoder/1057892 | **none** official. A community E38S6G5 mount (encoder geometry, unchecked): https://www.printables.com/model/666382-e38s6g5-type-incremental-rotary-encoder-mount. Measure housing Ø, boss Ø, hole circle and thread |
| **Drag chain** 10 × 10 mm | BOM "Schleppkette", "Kabelführung Encoder/Schlitten" | Black nylon chain beside the rail (Abb. 10/11/14) | Generic clone. Outer size differs by seller (15 × 19 or 14 × 18 mm), so measure | **online** for a near equivalent: igus 3D CAD for e-chains, free, no registration, https://www.igus.eu/energy-chain/configurator/3d-cad-models |
| **Motor bracket** 40 × 100 galvanised angle | BOM "Rechteckwinkel verzinkt", "Für die Montage vom Motor" | Angle bracket on the board, motor flange bolted through it (Abb. 3) | Hardware-store part (no link) | **none**. Measure thickness and hole pattern |
| **Motor damping**: rubber mat + rubber rings | §5.2 (fixed step loss), BOM rows "Gummimatte", "Gummiringe" | Grey/black rubber block under the motor (Abb. 3) | none | **none** |
| **Pendulum**: M3 threaded rod + M3 nuts as weight | §4.2, BOM "M3 Gewindestange", "Pendelgewicht" | Long rod with nuts at the top (Abb. 2) | DIN 976 M3 | **local**: `cad/fasteners/DIN913_Setscrew_M3x3/4.step` (pulley grub screws). Rod and nuts: FreeCAD Fasteners workbench |
| **Base board** 85 × 35 cm wood | §4, BOM "Grundplatte" | White board | none | Model as a plate. Thickness not given: measure |
| **Screws** | BOM: wood screws; machine screws M3/M4/M5 cap screws with nuts | — | — | FreeCAD Fasteners workbench (ISO 4762, ISO 4032) |
| **Enclosure** UNITEC surface junction box 241 × 180 × 95 | §4.7, BOM "Aufputz-Abzweigkasten" | Grey box, touchscreen cut-out in the lid (Abb. 14); DC jack on the side (§4.7) | No UNITEC listing at this size found online | **none**. Model as a box. Cut-outs: display (165 × 124 outline), switch Ø20.2, jack |

## 4. Electrical parts with mechanical interfaces

| Part | Datasheet / shop | CAD |
|---|---|---|
| Raspberry Pi 4 Model B | https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf, mechanical drawing **local** | **local**: `cad/raspberry_pi_4b/Raspberry_Pi_4B.step` (FreeCAD-library, 52 MB) |
| A4988 carrier (MakerMind clone of Pololu #1182) | **local**: Allegro datasheet + Pololu dimension drawing | none needed: 15.2 × 20.3 mm board, 0.1" pins |
| XY-3606 buck 24 → 5 V | shop page (§2), 36 × 64 mm | **none** |
| 7" touchscreen Joy-IT RB-LCD-7-3 | **local** datasheet, 165 × 124 × 13 mm | **none** |
| Rocker switch SCI R13-112 | reichelt (§2), UL/VDE certificates only | **local**: `cad/rocker_switch/RoundRockerSwitch2Pins.step` (generic Ø23, close match) |
| Fuse block Littelfuse 03500417TXN + 3 × 1 A ATO/FKS fuses | **local** datasheet | **none** found |
| 24 V 5 A desk PSU (SANPU, 5.5 × 2.5 mm plug) + IEC mains cable | leds24 (§2) | not needed (off-board) |
| DC panel jack 5.5 × 2.5 mm | BOM "DC-Steckerbuchse 2,5mm", no link | **none**. Measure the panel hole |
| 40-way ribbon to Dupont, 10 cm USB cable, terminals | BOM | not needed |

Removed from the final build (thesis §4.5, §5.1), listed for completeness: the first prototype's round-tube rail
with linear bearings (Abb. 12/17), the CD4093B Schmitt-trigger + 10 kHz RC filter board (Abb. 15), the rubber
coupling between the pendulum and the encoder shaft, and the Pi 3B+. The thin rubber layer under the encoder
holder may still be fitted (`thesis_notes.md`).

## 5. What to measure on the rig (no CAD source exists)

1. Rail length and the first hole's distance from each end (fixes discrepancy 1; feeds `make_mgn15_rail.py --length --e1`).
2. Motor pulley: flange Ø, hub Ø and length, belt-groove width.
3. Pendulum hub pulley: tooth count, flange Ø, hub length, which grub-screw hole holds the rod.
4. Encoder: housing Ø (thesis says 40, listings say 38–39), boss Ø and height, hole circle and thread of the 3 front holes.
5. All printed parts (cart, encoder holder, tensioner, switch blocks): or ask the author for the original files.
6. Motor angle bracket, rubber mat thickness, base board thickness, drag-chain link size and bend radius.
