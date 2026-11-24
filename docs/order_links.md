# Order links

Links for buying the Pendel rig's parts. The **BOM links** come from the thesis parts list (Anhang D); prices are
the thesis's own (2025/26). Every link was checked on 2026-11-11 unless marked otherwise. Part details and CAD
are in `parts_reference.md`.

## Parts with a link in the thesis

| Part | Qty | Link | Notes |
|---|---|---|---|
| 24 V power supply, 5 A | 1 | https://www.leds24.com/Netzteil-stabilisiert-24VDC-5A-120Watt | 34.90 €. 5.5 × 2.5 mm plug. Mains cable not included |
| Stepper motor ACT 23HS2430 (NEMA 23) | 1 | https://www.reichelt.de/de/de/shop/produkt/schrittmotor_4_pol_1_8_4_8_v_dc-229320 | 28.30 € |
| Raspberry Pi 4 B, 4 GB | 1 | https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_4_b_4x_1_8_ghz_4_gb_ram_wlan_bt-259920 | 64.50 € |
| GT2 timing belt, 6 mm, open (10 m roll) | 1 | https://www.roboter-bausatz.de/p/10-meter-gt2-zahnriemen-offen-6mm | 11.95 €. Need ≥ 1.5 m |
| A4988 stepper driver | 1 | https://www.roboter-bausatz.de/p/a4988-schrittmotorentreiber | 2.99 € |
| Linear rail **MGN15H, 600 mm**, with carriage | 1 | https://www.roboter-bausatz.de/p/linearfuehrung-mgn15h-600mm | 30.80 €. The rig has MGN15, so this is the right size; the thesis text's "MGN12" is wrong |
| 608-2Z bearing, 8 × 22 × 7 (belt idler) | 1 | https://3d-druckershop.com/p/fag-miniatur-rillenkugellager-608-zz-2z-8x22x7-mm-kaufen | **Not needed**: replaced by the toothed belt idler under Additions (decided 2026-11-19). 3.15 €. **Not checked**: the site's security certificate is broken. Any 608ZZ/608-2Z works |
| Round rocker switch, on/off | 1 | https://www.reichelt.de/de/de/shop/produkt/wippenschalter_rund_1x_ein_-_aus-105442 | 1.05 €. Panel hole 20.2 mm |
| Fuse holder, 5-way, ATO blade (Littelfuse 03500417TXN) | 1 | https://www.reichelt.de/de/de/shop/produkt/sicherungshalter_80_v_15_a-242449 | 15.50 € |
| 7" touchscreen, Joy-IT RB-LCD-7-3 | 1 | https://www.reichelt.com/ch/de/shop/produkt/raspberry_-_display_lcd-touch_7_ips-317040 | 64.85 € |
| Buck converter XY-3606, 24 V → 5 V | 1 | https://www.roboter-bausatz.de/p/spannungswandler-xy-3606-dc-dc-stepdown-modul-9-36v-auf-5v | 3.11 € |
| 40-way ribbon cable to Dupont, 25 cm | 1 | reichelt, "Raspberry Pi – Steckbrückenkabel 25 cm 40-polig auf Dupont" | 4.60 €. The link is cut off in the thesis, so search reichelt for that name |

Not needed: the thesis also links an MGN12C carriage
(https://3d-druckershop.com/p/mgn12c-linearfuehrungswagen-kaufen). Its own note says the rail already includes
a carriage, and an MGN12 carriage does not fit an MGN15 rail.

## Additions (not in the thesis)

| Part | Qty | Link | Notes |
|---|---|---|---|
| Axial fan SUNON MF40202V2-1000U-A99, 40 × 40 × 20 mm, 24 V | 1 | https://www.reichelt.de/de/de/shop/produkt/axialluefter_40x40x20mm_24v_13_1m_h_21dba-260665 | 3.40 € (2026-11-16). 30 mA / 0.68 W, 13.1 m³/h, 21 dB, 2 bare leads. Runs off the rig's 24 V. Datasheet: https://cdn-reichelt.de/documents/datenblatt/C800/MF40202V21000UA99_D04115160G-01.pdf |
| Belt idler, GT2, 20 teeth, 2 ball bearings, 5 mm bore, 6 mm belt | 1 | https://www.roboter-bausatz.de/p/gt2-riemenscheibe-20-zaehne-5mm-bohrung-mit-dual-kugellager | RBS12569, 1.75 €, "sofort verfügbar" (2026-11-19). Same shop as the belt. **Chosen** (2026-11-19) to replace the 608-2Z roller above; the idler's tooth count doesn't change steps/m (only the motor pulley does). Needs an M5 axle instead of the 8 mm one: reprint or sleeve the tensioner fork. Pitch Ø 12.7 mm vs ~24 mm for the 608 under the belt teeth, so the belt sits ~5.6 mm closer to the axle: check that the run clamped to the cart stays parallel to the rail. Largest bearing idler in the shop (2026-11-19): its 30–80-tooth GT2 pulleys all have set screws and no bearings. Reviews: oil the bearings, deburr the bore; one arrived with a bearing missing |

## Parts with no link in the thesis

Buy these by description. Where the rig needs a measurement first, it says so.

| Part | Qty | What to search for | Notes |
|---|---|---|---|
| Blade fuses, 1 A | 3 | "FKS / ATO Flachstecksicherung 1 A" | Fits the fuse holder above |
| Rotary encoder, 600 P/R | 1 | "E38S6G5-600B-G24N" | NPN open collector, 5–24 V, 6 mm shaft. The thesis prints "B38S6G5"; listings all use E. 18.99 € in the thesis |
| Limit switches, Omron V-166-1C5 | 2 | https://www.reichelt.de/de/de/shop/produkt/mikroschalter_rollenhebel_mittel-243170 | 2.42 € each, in stock (2026-11-16). Genuine Omron, same V-size body (27.8 × 15.9 × 10.3 mm, same 2 holes) and hinge roller lever as the rig's Heschen V-156-1C25; 16 A instead of 15 A, lighter operating force (1.96 N). SPDT with 6.3 mm tabs: wire COM + NO as before. Reichelt does not carry the exact V-156-1C25 |
| Motor pulley, GT2, 60 teeth, 8 mm bore, 6 mm belt | 1 | https://www.roboter-bausatz.de/p/gt2-riemenscheibe-60-zaehne-8mm-bohrung-fuer-6mm-zahnriemen | RBS12574, 4.69 €, "sofort verfügbar" (2026-11-16). Same shop as the belt |
| Pendulum hub pulley, GT2, 20 teeth, 6.00 mm bore | 1 | https://www.dold-mechatronik.de/Toothed-belt-GT2-6mm-wide-20-teeth,-bore-6.00mm-H7-with-clamping-screws | Dold Mechatronik art. 46101, 2.90 €, 6.00 mm H7 bore, 2 clamping screws at 120°. Stock and screw size not shown to me: the pendulum rod (M3) screws into a clamping-screw hole, so confirm the holes are M3. It doesn't carry the belt, so the tooth count only changes the hub size. Most 6 mm-bore GT2 pulleys in German shops are actually 6.35 mm (1/4"): those wobble on the 6 mm encoder shaft |
| Drag chain (cable carrier, "Schleppkette"), 10 × 10 mm inside, R28, 1 m | 1 | "Schleppkette 10x10 R28 1m". Examples: https://funduino.com/en/products/drag-chain-schleppkette-kabelfuhrung-kabelkanal-r28-10x10mm (DE; 45 links + 2 end pieces, ~11.48 €, page not opened: rate-limited), https://www.bastelgarage.ch/10x10mm-nylon-cable-carrier-1m (CH; checked: 18 × 15 mm outside, R28, end pieces included, CHF 13.90) | The black plastic link chain that keeps the encoder cable out of the belt. 1 m is enough (half the travel plus the bend loop). Links from different sellers may not snap together, so buy a whole chain rather than spare links |
| DC panel jack, 5.5 × 2.5 mm | 1 | "DC Einbaubuchse 5,5 x 2,5" | Must match the PSU plug |
| Mains cable (IEC / Kaltgerätekabel) | 1 | match the PSU's inlet | Not included with the PSU |
| Enclosure, UNITEC surface junction box, 241 × 180 × 95 mm | 1 | "Aufputz-Abzweigkasten 241 x 180 x 95" | Hardware store (Bauhaus/OBI). I found no online listing at this size |
| Galvanised angle bracket, 40 × 100 | 1 | "Winkelverbinder verzinkt 40 x 100" | Holds the motor. Hardware store |
| Rubber mat + rubber rings | 1 each | rubber mat, rubber O-rings / grommets | Motor vibration damping |
| M3 threaded rod + M3 nuts | 1 + some | "Gewindestange M3" | Pendulum rod; the nuts are the weight |
| Cap screws M3/M4/M5 with nuts, wood screws | 1 set each | "Zylinderschrauben Set M3 M4 M5" | |
| Pull-up resistors 8 kΩ ×4, capacitor ≥ 100 µF, terminals | — | — | Only for the thesis wiring. The new PCB uses 4.7 kΩ / 10 kΩ |
| USB cable, 10 cm | 1 | short USB-A to USB-C | Buck converter → Pi |
| Base board, 85 × 35 cm | 1 | wood board | |
| 3D-printed parts (cart, encoder holder, 2-part belt tensioner, switch brackets) | — | — | Not buyable, and the files were never published: reprint from measurements or ask the author |
