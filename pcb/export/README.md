# PCB exports

Generated on 2026-11-23 from `pcb/PCB.kicad_sch` and `pcb/PCB.kicad_pcb` with KiCad 10.0.8's command line. The design
was saved with KiCad 10, so KiCad 9 cannot open it. These files are copies: after any change to the design, export them
again.

Source check: the maintainer confirmed `PCB_6_10_2026.zip` as the latest design. Its design files (board, schematic,
project, library tables) are byte-identical to `pcb/` here (board saved 2026-10-06, schematic 2026-10-05). Exporting
straight from the zip gave the same BOM and Gerbers; only the creation timestamps in the file headers differ.

| file | what it is |
|---|---|
| `PCB_schematic.pdf` | the schematic |
| `PCB_bom.csv` | bill of materials: refs, value, footprint, quantity |
| `PCB_layout.pdf` | F.Cu, B.Cu, F.Silkscreen and Edge.Cuts on one page, for reading the board |
| `gerbers/` | every layer as Gerber (`PCB-*.g*`), the drill file `PCB.drl` and the job file `PCB-job.gbrjob`; zip this folder for a board house |

The silkscreen and the layout PDF use the font "HSBI Lausanne 200", which is not installed on the exporting PC. KiCad
substituted Verdana, so text on those layers looks different from the original design. Copper, mask, drill and outline
are not affected.

Regenerate (Windows, Git Bash, from the repo root):

```
K="/c/Program Files/KiCad/10.0/bin/kicad-cli.exe"
"$K" sch export pdf -o pcb/export/PCB_schematic.pdf pcb/PCB.kicad_sch
"$K" sch export bom -o pcb/export/PCB_bom.csv pcb/PCB.kicad_sch
"$K" pcb export gerbers -o pcb/export/gerbers/ pcb/PCB.kicad_pcb
"$K" pcb export drill -o pcb/export/gerbers/ pcb/PCB.kicad_pcb
"$K" pcb export pdf --layers F.Cu,B.Cu,F.Silkscreen,Edge.Cuts --mode-single -o pcb/export/PCB_layout.pdf pcb/PCB.kicad_pcb
```
