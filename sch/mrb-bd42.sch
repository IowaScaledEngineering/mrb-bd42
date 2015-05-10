v 20130925 2
C 53400 56700 1 0 1 rs485-1.sym
{
T 51750 58500 5 10 0 0 0 6 1
device=SP485REN
T 52050 56850 5 10 1 1 0 6 1
refdes=U2
T 51750 58300 5 10 0 0 0 6 1
footprint=SO8
}
T 42200 60000 9 10 1 0 0 6 1
12V
T 42200 59600 9 10 1 0 0 6 1
GND
T 42200 59100 9 10 1 0 0 6 1
RS485-A
T 42200 58700 9 10 1 0 0 6 1
RS485-B
N 43100 60100 45900 60100 4
N 43100 59700 44200 59700 4
N 44200 59700 44200 58800 4
N 48000 59000 48000 58800 4
N 48000 59900 48000 60100 4
N 50000 60100 57800 60100 4
C 49100 58300 1 0 0 gnd-1.sym
C 56100 45300 1 0 0 crystal-1.sym
{
T 56300 45800 5 10 0 0 0 0 1
device=CRYSTAL
T 56300 45600 5 10 1 1 0 0 1
refdes=Y1
T 56300 46000 5 10 0 0 0 0 1
symversion=0.1
T 56450 45100 5 10 1 1 0 3 1
value=20MHz
T 56100 45300 5 10 0 1 0 0 1
footprint=crystal-hc49-smt
}
N 56100 44900 56100 49900 4
N 56800 44900 56800 49600 4
C 56300 44900 1 90 1 capacitor-1.sym
{
T 55600 44700 5 10 0 1 270 2 1
device=CAPACITOR
T 56000 44600 5 10 1 1 0 6 1
refdes=C7
T 55400 44700 5 10 0 0 270 2 1
symversion=0.1
T 56000 44100 5 10 1 1 0 6 1
value=22pF
T 56300 44900 5 10 0 0 0 6 1
footprint=0805
}
C 56000 43700 1 0 0 gnd-1.sym
C 56600 44900 1 270 0 capacitor-1.sym
{
T 57300 44700 5 10 0 1 270 0 1
device=CAPACITOR
T 56900 44600 5 10 1 1 0 0 1
refdes=C8
T 57500 44700 5 10 0 0 270 0 1
symversion=0.1
T 56900 44100 5 10 1 1 0 0 1
value=22pF
T 56600 44900 5 10 0 0 0 0 1
footprint=0805
}
C 56700 43700 1 0 0 gnd-1.sym
N 43100 59200 43800 59200 4
N 43800 58100 43800 59200 4
N 43800 58100 51800 58100 4
N 43100 58800 43400 58800 4
N 43400 58800 43400 57300 4
N 43400 57300 51800 57300 4
N 54300 58000 54300 58800 4
N 53400 58000 55000 58000 4
C 52500 55600 1 0 0 gnd-1.sym
N 52600 55900 52600 56700 4
C 54400 58800 1 90 0 resistor-1.sym
{
T 54000 59100 5 10 0 0 90 0 1
device=RESISTOR
T 54100 59000 5 10 1 1 90 0 1
refdes=R3
T 54600 59000 5 10 1 1 90 0 1
value=10k
T 54400 58800 5 10 0 0 90 0 1
footprint=0805
}
N 54300 59700 54300 60100 4
N 53400 57700 54600 57700 4
C 54000 56200 1 90 0 resistor-1.sym
{
T 53600 56500 5 10 0 0 90 0 1
device=RESISTOR
T 53700 56400 5 10 1 1 90 0 1
refdes=R4
T 54200 56400 5 10 1 1 90 0 1
value=10k
T 54000 56200 5 10 0 0 90 0 1
footprint=0805
}
C 55500 58800 1 90 0 resistor-1.sym
{
T 55100 59100 5 10 0 0 90 0 1
device=RESISTOR
T 55200 59000 5 10 1 1 90 0 1
refdes=R2
T 55700 59000 5 10 1 1 90 0 1
value=330
T 55500 58800 5 10 0 0 90 0 1
footprint=0805
}
N 55400 59700 55400 60100 4
C 55600 57800 1 90 0 led-3.sym
{
T 55850 57650 5 10 1 1 90 0 1
device=AMBER LED
T 55050 58250 5 10 1 1 90 0 1
refdes=D2
T 55600 57800 5 10 0 0 0 0 1
footprint=0805
}
N 55400 58800 55400 58700 4
N 55400 48700 55400 57800 4
N 52600 58600 52600 60100 4
T 66900 39100 9 10 1 0 0 0 1
MRBus 4 Channel DCC Block Detector 3
T 66700 38800 9 10 1 0 0 0 1
mrb-bd42.sch
T 66900 38500 9 10 1 0 0 0 1
1
T 68400 38500 9 10 1 0 0 0 1
1
T 70600 38500 9 10 1 0 0 0 1
Nathan D. Holmes
T 70600 38800 9 10 1 0 0 0 1
$Revision: 82 $
T 67500 40500 9 10 1 0 0 2 3
Notes:
1) All caps X5R or X7R, 6.3V or better ceramic unless otherwise noted.

C 56800 58800 1 90 0 resistor-1.sym
{
T 56400 59100 5 10 0 0 90 0 1
device=RESISTOR
T 56500 59000 5 10 1 1 90 0 1
refdes=R1
T 57000 59000 5 10 1 1 90 0 1
value=330
T 56800 58800 5 10 0 0 90 0 1
footprint=0805
}
N 56700 59700 56700 60100 4
C 56900 57800 1 90 0 led-3.sym
{
T 57150 57750 5 10 1 1 90 0 1
device=GREEN LED
T 56350 58250 5 10 1 1 90 0 1
refdes=D1
T 56900 57800 5 10 0 0 0 0 1
footprint=0805
}
C 56600 57400 1 0 0 gnd-1.sym
N 56700 57700 56700 57800 4
C 43100 58600 1 0 1 termblk2-1.sym
{
T 42100 59250 5 10 0 0 0 6 1
device=TERMBLK2
T 42700 58400 5 10 1 1 0 6 1
refdes=J2
T 43100 58600 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 43100 59500 1 0 1 termblk2-1.sym
{
T 42100 60150 5 10 0 0 0 6 1
device=TERMBLK2
T 42700 60400 5 10 1 1 0 6 1
refdes=J1
T 43100 59500 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 68200 40800 1 0 0 hole-1.sym
{
T 68200 40800 5 10 0 1 0 0 1
device=HOLE
T 68400 41400 5 10 1 1 0 4 1
refdes=H1
T 68200 40800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 68700 40800 1 0 0 hole-1.sym
{
T 68700 40800 5 10 0 1 0 0 1
device=HOLE
T 68900 41400 5 10 1 1 0 4 1
refdes=H2
T 68700 40800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 69200 40800 1 0 0 hole-1.sym
{
T 69200 40800 5 10 0 1 0 0 1
device=HOLE
T 69400 41400 5 10 1 1 0 4 1
refdes=H3
T 69200 40800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 69700 40800 1 0 0 hole-1.sym
{
T 69700 40800 5 10 0 1 0 0 1
device=HOLE
T 69900 41400 5 10 1 1 0 4 1
refdes=H4
T 69700 40800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 46800 60100 48400 60100 4
N 49200 59500 49200 58800 4
C 50100 59900 1 270 0 capacitor-1.sym
{
T 50800 59700 5 10 0 1 270 0 1
device=CAPACITOR
T 50400 59600 5 10 1 1 0 0 1
refdes=C2
T 51000 59700 5 10 0 0 270 0 1
symversion=0.1
T 50400 59100 5 10 1 1 0 0 1
value=1uF
T 50100 59900 5 10 0 0 0 0 1
footprint=0805
}
N 50300 59900 50300 60100 4
N 50300 59000 50300 58800 4
N 53400 57400 55400 57400 4
C 41500 52500 1 0 0 current-transformer.sym
{
T 41900 53900 5 10 1 1 0 0 1
refdes=T1
T 41800 53800 5 10 0 0 0 0 1
device=transformer
T 41500 52500 5 10 0 0 0 0 1
footprint=CST306
}
N 42900 53700 46100 53700 4
{
T 45400 54100 5 10 1 1 0 0 1
netname=VBIAS
}
N 44700 53600 44700 53700 4
N 43300 53700 43300 53600 4
N 42900 52700 44900 52700 4
N 53900 57100 53900 57700 4
N 52600 55900 53900 55900 4
N 53900 55900 53900 56200 4
N 53400 57100 53400 55900 4
N 44200 58800 50300 58800 4
N 49200 58800 49200 58600 4
C 51800 60400 1 0 0 5V-plus-1.sym
N 52000 60400 52000 60100 4
C 58100 45800 1 0 0 mega48-tqfp32.sym
{
T 62600 52300 5 10 1 1 0 6 1
refdes=U6
T 58400 52600 5 10 0 0 0 0 1
device=ATMega48-TQFP32
T 58400 52800 5 10 0 0 0 0 1
footprint=TQFP32_7
}
N 60300 53100 64800 53100 4
N 58100 49600 56800 49600 4
N 58100 49900 56100 49900 4
N 58100 48700 55400 48700 4
N 58100 48400 55000 48400 4
N 55000 48400 55000 58000 4
N 58100 48100 54600 48100 4
N 54600 48100 54600 57700 4
C 58300 55600 1 0 0 avrprog-1.sym
{
T 58300 57200 5 10 0 1 0 0 1
device=AVRPROG
T 58900 56900 5 10 1 1 0 0 1
refdes=J3
T 58300 55600 5 10 0 0 0 0 1
footprint=JUMPER3x2
}
N 59700 56600 60000 56600 4
N 60000 56600 60000 57100 4
N 62900 46600 66000 46600 4
C 59600 55400 1 0 0 gnd-1.sym
N 59700 55800 59700 55700 4
C 60400 45200 1 0 0 gnd-1.sym
N 60300 45800 60300 45500 4
N 60300 45500 60700 45500 4
N 60700 45800 60700 45500 4
C 66100 47200 1 90 0 resistor-1.sym
{
T 65700 47500 5 10 0 0 90 0 1
device=RESISTOR
T 65800 47400 5 10 1 1 90 0 1
refdes=R18
T 66300 47400 5 10 1 1 90 0 1
value=10k
T 66100 47200 5 10 0 0 90 0 1
footprint=0805
}
C 65800 46400 1 270 0 capacitor-1.sym
{
T 66500 46200 5 10 0 1 270 0 1
device=CAPACITOR
T 66100 46100 5 10 1 1 0 0 1
refdes=C9
T 66700 46200 5 10 0 0 270 0 1
symversion=0.1
T 66100 45600 5 10 1 1 0 0 1
value=1uF
T 65800 46400 5 10 0 0 0 0 1
footprint=0805
}
C 65900 45000 1 0 0 gnd-1.sym
C 63900 50400 1 0 0 gnd-1.sym
N 62900 50800 64000 50800 4
N 66000 45500 66000 45300 4
C 63800 51700 1 270 0 capacitor-1.sym
{
T 64500 51500 5 10 0 1 270 0 1
device=CAPACITOR
T 64100 51400 5 10 1 1 0 0 1
refdes=C10
T 64700 51500 5 10 0 0 270 0 1
symversion=0.1
T 64100 50900 5 10 1 1 0 0 1
value=0.1uF
T 63800 51700 5 10 0 0 0 0 1
footprint=0805
}
N 64000 50800 64000 50700 4
N 62900 51400 62900 51700 4
N 60700 53100 60700 52500 4
N 62900 51100 63400 51100 4
N 63400 51100 63400 53100 4
N 66000 46400 66000 47200 4
C 72400 50200 1 0 1 modular_6p6c_small.sym
{
T 72200 52000 5 10 1 1 0 6 1
refdes=J4
T 72400 54225 5 10 0 0 0 6 1
footprint=modular_6p6c_lp.fp
T 72200 51800 5 10 1 1 0 6 1
device=RJ11
T 72400 50200 5 10 0 0 0 0 1
footprint=modular_6p6c_lp
}
C 72400 47200 1 0 1 modular_6p6c_small.sym
{
T 72200 49000 5 10 1 1 0 6 1
refdes=J5
T 72400 51225 5 10 0 0 0 6 1
footprint=modular_6p6c_lp.fp
T 72200 48800 5 10 1 1 0 6 1
device=RJ11
T 72400 47200 5 10 0 0 0 0 1
footprint=modular_6p6c_lp
}
C 69700 46700 1 0 0 gnd-1.sym
C 69700 49700 1 0 0 gnd-1.sym
N 69800 47000 69800 47200 4
N 69800 47200 70000 47200 4
N 69800 50000 69800 50200 4
N 69800 50200 70000 50200 4
C 69600 54500 1 0 0 5V-plus-1.sym
C 69600 48900 1 0 0 5V-plus-1.sym
N 69800 48900 69800 48700 4
N 69800 48700 70000 48700 4
N 69800 51700 70000 51700 4
N 68600 51400 70000 51400 4
{
T 68800 51400 5 10 1 1 0 0 1
netname=IN1-1
}
N 68600 51100 70000 51100 4
{
T 68800 51100 5 10 1 1 0 0 1
netname=IN1-2
}
N 68600 50800 70000 50800 4
{
T 68800 50800 5 10 1 1 0 0 1
netname=IN1-3
}
N 62900 51700 64000 51700 4
C 51400 59700 1 270 0 capacitor-1.sym
{
T 52100 59500 5 10 0 1 270 0 1
device=CAPACITOR
T 51700 59400 5 10 1 1 0 0 1
refdes=C15
T 52300 59500 5 10 0 0 270 0 1
symversion=0.1
T 51700 58900 5 10 1 1 0 0 1
value=0.1uF
T 51400 59700 5 10 0 0 0 0 1
footprint=0805
}
C 51500 58400 1 0 0 gnd-1.sym
C 47800 59900 1 270 0 Cap_H-2.sym
{
T 48100 59600 5 10 1 1 0 0 1
refdes=C1
T 49300 59900 5 10 0 0 270 0 1
device=Capacitor
T 47300 59600 5 10 1 1 0 2 1
value=68uF
T 47800 59900 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 46900 59200 5 10 1 1 0 0 1
description=Electrolytic
}
C 48400 59500 1 0 0 lm7805-1.sym
{
T 50000 60800 5 10 0 0 0 0 1
device=7805
T 49800 60500 5 10 1 1 0 6 1
refdes=U1
T 48400 59500 5 10 0 1 0 0 1
footprint=TO220
}
C 40800 38200 0 0 0 title-bordered-A1.sym
N 56700 58700 56700 58800 4
C 42400 55000 1 0 0 rj45-1.sym
{
T 42400 57900 5 10 0 0 0 0 1
device=RJ45
T 41800 54700 5 10 1 1 0 0 1
footprint=modular_8p8c_lp.fp
T 42400 56900 5 10 1 1 0 0 1
refdes=J2A
}
N 43300 56600 44800 56600 4
N 43300 56200 44800 56200 4
N 43300 56400 43700 56400 4
N 43700 54700 43700 56400 4
N 43700 55600 43300 55600 4
C 43600 54400 1 0 0 gnd-1.sym
N 43300 56000 44000 56000 4
N 43300 55800 44400 55800 4
N 44800 56200 44800 60100 4
N 44000 56000 44000 58100 4
N 44400 55800 44400 57300 4
C 45900 59900 1 0 0 mbr0540-1.sym
{
T 45722 59572 5 10 1 1 0 0 1
device=MBR0520LT1G
T 46200 60400 5 10 1 1 0 0 1
refdes=D9
T 46241 60732 5 10 0 1 0 0 1
footprint=SOD123
}
C 44800 52700 1 90 0 resistor-1.sym
{
T 44400 53000 5 10 0 0 90 0 1
device=RESISTOR
T 44500 53100 5 10 1 1 90 0 1
refdes=R9
T 45000 53100 5 10 1 1 90 0 1
value=1k
T 44800 52700 5 10 0 0 90 0 1
footprint=0805
}
N 45000 53700 45000 54200 4
N 45000 54200 45300 54200 4
N 45800 52700 46800 52700 4
N 46100 52700 46100 53300 4
N 47700 52700 48400 52700 4
N 48400 52700 48400 53500 4
N 48400 53500 49000 53500 4
C 49400 52600 1 0 0 gnd-1.sym
N 49000 53100 49000 51900 4
N 50200 51900 50200 53300 4
N 47900 51900 47700 51900 4
{
T 47100 51800 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 53300 52300 53300 4
{
T 52400 53200 5 10 1 1 0 0 1
netname=IDET1
}
C 51700 52100 1 0 0 gnd-1.sym
C 49300 53700 1 0 0 5V-plus-1.sym
N 62900 48400 64300 48400 4
{
T 64400 48300 5 10 1 1 0 0 1
netname=IDET1
}
N 62900 48100 64300 48100 4
{
T 64400 48000 5 10 1 1 0 0 1
netname=IDET2
}
N 62900 47800 64300 47800 4
{
T 64400 47700 5 10 1 1 0 0 1
netname=IDET3
}
N 62900 47500 64300 47500 4
{
T 64400 47400 5 10 1 1 0 0 1
netname=IDET4
}
C 44500 52600 1 0 0 res-pack4-1.sym
{
T 44500 52600 5 10 0 0 0 0 1
slot=2
T 45300 52400 5 10 1 1 0 0 1
value=10k
T 44700 52200 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 45000 52400 5 10 1 1 0 0 1
refdes=R5
}
C 48100 52600 1 0 1 res-pack4-1.sym
{
T 48100 52600 5 10 0 0 0 6 1
slot=1
T 47300 52400 5 10 1 1 0 6 1
value=10k
T 47900 52200 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 47600 52400 5 10 1 1 0 6 1
refdes=R5
}
C 46100 53100 1 0 0 tsv914-1.sym
{
T 46925 53250 5 8 0 0 0 0 1
device=TSV914
T 46400 53400 5 10 1 1 0 0 1
refdes=U4
T 45400 53400 5 10 1 1 0 0 1
device=TSV914
T 46100 53100 5 10 0 0 0 0 1
footprint=SO14
T 46100 53100 5 10 0 0 0 0 1
slot=4
}
C 49000 52900 1 0 0 tsv914-1.sym
{
T 49825 53050 5 8 0 0 0 0 1
device=TSV914
T 49300 53200 5 10 0 1 0 0 1
slot=3
T 49300 53200 5 10 1 1 0 0 1
refdes=U4
T 49000 52900 5 10 0 0 0 0 1
footprint=SO14
}
C 46400 55200 1 0 0 5V-plus-1.sym
C 46500 52800 1 0 0 gnd-1.sym
N 50300 53300 50000 53300 4
N 48800 51900 49200 51900 4
C 47500 51800 1 0 0 res-pack2-1.sym
{
T 47995 51600 5 10 1 1 0 0 1
refdes=R6
T 47500 51800 5 10 0 0 0 0 1
slot=1
T 47700 51400 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 51600 5 10 1 1 0 0 1
value=1k
}
C 48800 51800 1 0 0 res-pack2-1.sym
{
T 49395 51600 5 10 1 1 0 0 1
refdes=R7
T 48800 51800 5 10 0 0 0 0 1
slot=1
T 49700 51600 5 10 1 1 0 0 1
value=100k
T 49100 51400 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 51900 50200 51900 4
C 42900 53600 1 270 0 mmbd4448dw-1.sym
{
T 43500 53200 5 10 0 0 270 0 1
device=MMBD4448DW
T 43200 52500 5 10 1 1 0 0 1
refdes=D3
T 42898 53605 5 10 0 1 270 0 1
footprint=SOT363
T 42900 53600 5 10 0 0 0 0 1
slot=1
}
C 43500 52700 1 270 1 mmbd4448dw-1.sym
{
T 44100 53100 5 10 0 0 90 2 1
device=MMBD4448DW
T 43800 52600 5 10 1 1 180 6 1
refdes=D3
T 43498 52695 5 10 0 1 90 2 1
footprint=SOT363
T 43500 52700 5 10 0 0 180 6 1
slot=2
}
N 43900 53600 43900 53700 4
C 47500 53100 1 0 0 mmbd4448dw-1.sym
{
T 47900 53700 5 10 0 0 0 0 1
device=MMBD4448DW
T 47900 54000 5 10 1 1 0 0 1
refdes=D4
T 47495 53098 5 10 0 1 0 0 1
footprint=SOT363
T 47500 53100 5 10 0 0 0 0 1
slot=1
}
N 47500 53500 47100 53500 4
C 49900 53200 1 0 0 res-pack4-1.sym
{
T 49900 53200 5 10 0 0 0 0 1
slot=1
T 50800 53800 5 10 1 1 0 0 1
value=10k
T 50200 53600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 53800 5 10 1 1 0 0 1
refdes=R8
}
C 41500 48900 1 0 0 current-transformer.sym
{
T 41800 50200 5 10 0 0 0 0 1
device=transformer
T 41500 48900 5 10 0 0 0 0 1
footprint=CST306
T 41900 50300 5 10 1 1 0 0 1
refdes=T2
}
N 42900 50100 46100 50100 4
N 44700 50000 44700 50100 4
N 43300 50100 43300 50000 4
N 42900 49100 44900 49100 4
C 44800 49100 1 90 0 resistor-1.sym
{
T 44400 49400 5 10 0 0 90 0 1
device=RESISTOR
T 44800 49100 5 10 0 0 90 0 1
footprint=0805
T 44500 49500 5 10 1 1 90 0 1
refdes=R10
T 45000 49500 5 10 1 1 90 0 1
value=1k
}
N 45000 50100 45000 50600 4
N 45000 50600 45400 50600 4
{
T 45500 50500 5 10 1 1 0 0 1
netname=VBIAS
}
N 45800 49100 46800 49100 4
N 46100 49100 46100 49700 4
N 47700 49100 48400 49100 4
N 48400 49100 48400 49900 4
N 48400 49900 49000 49900 4
C 49400 49000 1 0 0 gnd-1.sym
N 49000 49500 49000 48300 4
N 50200 48300 50200 49700 4
N 47900 48300 47700 48300 4
{
T 47100 48200 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 49700 52300 49700 4
{
T 52400 49600 5 10 1 1 0 0 1
netname=IDET2
}
C 51700 48500 1 0 0 gnd-1.sym
C 49300 50100 1 0 0 5V-plus-1.sym
C 44500 49000 1 0 0 res-pack4-1.sym
{
T 44500 49000 5 10 0 0 0 0 1
slot=4
T 45300 48800 5 10 1 1 0 0 1
value=10k
T 44700 48600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 45000 48800 5 10 1 1 0 0 1
refdes=R5
}
C 48100 49000 1 0 1 res-pack4-1.sym
{
T 48100 49000 5 10 0 0 0 6 1
slot=3
T 47300 48800 5 10 1 1 0 6 1
value=10k
T 47900 48600 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 47600 48800 5 10 1 1 0 6 1
refdes=R5
}
C 46100 49500 1 0 0 tsv914-1.sym
{
T 46925 49650 5 8 0 0 0 0 1
device=TSV914
T 46400 49800 5 10 1 1 0 0 1
refdes=U4
T 45400 49800 5 10 1 1 0 0 1
device=TSV914
T 46100 49500 5 10 0 1 0 0 1
slot=1
T 46100 49500 5 10 0 0 0 0 1
footprint=SO14
}
C 49000 49300 1 0 0 tsv914-1.sym
{
T 49825 49450 5 8 0 0 0 0 1
device=TSV914
T 49300 49600 5 10 0 1 0 0 1
slot=2
T 49300 49600 5 10 1 1 0 0 1
refdes=U4
T 49000 49300 5 10 0 0 0 0 1
footprint=SO14
}
C 46500 49200 1 0 0 gnd-1.sym
N 50300 49700 50000 49700 4
N 48800 48300 49200 48300 4
C 47500 48200 1 0 0 res-pack2-1.sym
{
T 47500 48200 5 10 0 0 0 0 1
slot=2
T 47995 48000 5 10 1 1 0 0 1
refdes=R6
T 47700 47800 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 48000 5 10 1 1 0 0 1
value=1k
}
C 48800 48200 1 0 0 res-pack2-1.sym
{
T 48800 48200 5 10 0 0 0 0 1
slot=2
T 49395 48000 5 10 1 1 0 0 1
refdes=R7
T 49700 48000 5 10 1 1 0 0 1
value=100k
T 49100 47800 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 48300 50200 48300 4
C 42900 50000 1 270 0 mmbd4448dw-1.sym
{
T 43500 49600 5 10 0 0 270 0 1
device=MMBD4448DW
T 42898 50005 5 10 0 1 270 0 1
footprint=SOT363
T 43200 48900 5 10 1 1 0 0 1
refdes=D5
T 42900 50000 5 10 0 0 0 0 1
slot=1
}
C 43500 49100 1 270 1 mmbd4448dw-1.sym
{
T 44100 49500 5 10 0 0 90 2 1
device=MMBD4448DW
T 43498 49095 5 10 0 1 90 2 1
footprint=SOT363
T 43500 49100 5 10 0 0 180 6 1
slot=2
T 43800 49000 5 10 1 1 180 6 1
refdes=D5
}
N 43900 50000 43900 50100 4
C 47500 49500 1 0 0 mmbd4448dw-1.sym
{
T 47900 50100 5 10 0 0 0 0 1
device=MMBD4448DW
T 47495 49498 5 10 0 1 0 0 1
footprint=SOT363
T 47900 50400 5 10 1 1 0 0 1
refdes=D4
T 47500 49500 5 10 0 1 0 0 1
slot=2
}
N 47500 49900 47100 49900 4
C 49900 49600 1 0 0 res-pack4-1.sym
{
T 49900 49600 5 10 0 0 0 0 1
slot=2
T 50800 50200 5 10 1 1 0 0 1
value=10k
T 50200 50000 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 50200 5 10 1 1 0 0 1
refdes=R8
}
C 46400 50300 1 0 0 5V-plus-1.sym
C 41500 44900 1 0 0 current-transformer.sym
{
T 41800 46200 5 10 0 0 0 0 1
device=transformer
T 41500 44900 5 10 0 0 0 0 1
footprint=CST306
T 41900 46300 5 10 1 1 0 0 1
refdes=T3
}
N 42900 46100 46100 46100 4
{
T 45400 46500 5 10 1 1 0 0 1
netname=VBIAS
}
N 44700 46000 44700 46100 4
N 43300 46100 43300 46000 4
N 42900 45100 44900 45100 4
C 44800 45100 1 90 0 resistor-1.sym
{
T 44400 45400 5 10 0 0 90 0 1
device=RESISTOR
T 44800 45100 5 10 0 0 90 0 1
footprint=0805
T 44500 45500 5 10 1 1 90 0 1
refdes=R21
T 45000 45500 5 10 1 1 90 0 1
value=1k
}
N 45000 46100 45000 46600 4
N 45000 46600 45300 46600 4
N 45800 45100 46800 45100 4
N 46100 45100 46100 45700 4
N 47700 45100 48400 45100 4
N 48400 45100 48400 45900 4
N 48400 45900 49000 45900 4
C 49400 45000 1 0 0 gnd-1.sym
N 49000 45500 49000 44300 4
N 50200 44300 50200 45700 4
N 47900 44300 47700 44300 4
{
T 47100 44200 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 45700 52300 45700 4
{
T 52400 45600 5 10 1 1 0 0 1
netname=IDET3
}
C 51700 44500 1 0 0 gnd-1.sym
C 49300 46100 1 0 0 5V-plus-1.sym
C 46200 45000 1 0 1 res-pack4-1.sym
{
T 46200 45000 5 10 0 0 0 6 1
slot=1
T 45300 44800 5 10 1 1 0 6 1
value=10k
T 46000 44600 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 45700 44800 5 10 1 1 0 6 1
refdes=R11
}
C 46400 45000 1 0 0 res-pack4-1.sym
{
T 46400 45000 5 10 0 0 0 0 1
slot=2
T 47300 44800 5 10 1 1 0 0 1
value=10k
T 46600 44600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 46900 44800 5 10 1 1 0 0 1
refdes=R11
}
C 46100 45500 1 0 0 tsv914-1.sym
{
T 46925 45650 5 8 0 0 0 0 1
device=TSV914
T 46400 45800 5 10 1 1 0 0 1
refdes=U5
T 45400 45800 5 10 1 1 0 0 1
device=TSV914
T 46100 45500 5 10 0 0 0 0 1
footprint=SO14
}
C 49000 45300 1 0 0 tsv914-1.sym
{
T 49825 45450 5 8 0 0 0 0 1
device=TSV914
T 49300 45600 5 10 0 1 0 0 1
slot=2
T 49300 45600 5 10 1 1 0 0 1
refdes=U5
T 49000 45300 5 10 0 0 0 0 1
footprint=SO14
}
C 46500 45200 1 0 0 gnd-1.sym
N 50300 45700 50000 45700 4
N 48800 44300 49200 44300 4
C 47500 44200 1 0 0 res-pack2-1.sym
{
T 47500 44200 5 10 0 0 0 0 1
slot=1
T 47995 44000 5 10 1 1 0 0 1
refdes=R12
T 47700 43800 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 44000 5 10 1 1 0 0 1
value=1k
}
C 48800 44200 1 0 0 res-pack2-1.sym
{
T 48800 44200 5 10 0 0 0 0 1
slot=1
T 49195 44000 5 10 1 1 0 0 1
refdes=R13
T 49700 44000 5 10 1 1 0 0 1
value=100k
T 49100 43800 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 44300 50200 44300 4
C 42900 46000 1 270 0 mmbd4448dw-1.sym
{
T 43500 45600 5 10 0 0 270 0 1
device=MMBD4448DW
T 42898 46005 5 10 0 1 270 0 1
footprint=SOT363
T 43200 44900 5 10 1 1 0 0 1
refdes=D7
T 42900 46000 5 10 0 0 0 0 1
slot=1
}
C 43500 45100 1 270 1 mmbd4448dw-1.sym
{
T 44100 45500 5 10 0 0 90 2 1
device=MMBD4448DW
T 43498 45095 5 10 0 1 90 2 1
footprint=SOT363
T 43500 45100 5 10 0 0 180 6 1
slot=2
T 43800 45000 5 10 1 1 180 6 1
refdes=D7
}
N 43900 46000 43900 46100 4
C 47500 45500 1 0 0 mmbd4448dw-1.sym
{
T 47900 46100 5 10 0 0 0 0 1
device=MMBD4448DW
T 47495 45498 5 10 0 1 0 0 1
footprint=SOT363
T 47900 46400 5 10 1 1 0 0 1
refdes=D6
T 47500 45500 5 10 0 0 0 0 1
slot=2
}
N 47500 45900 47100 45900 4
C 49900 45600 1 0 0 res-pack4-1.sym
{
T 49900 45600 5 10 0 0 0 0 1
slot=3
T 50800 46200 5 10 1 1 0 0 1
value=10k
T 50200 46000 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 46200 5 10 1 1 0 0 1
refdes=R8
}
C 41500 41300 1 0 0 current-transformer.sym
{
T 41800 42600 5 10 0 0 0 0 1
device=transformer
T 41500 41300 5 10 0 0 0 0 1
footprint=CST306
T 41900 42700 5 10 1 1 0 0 1
refdes=T4
}
N 42900 42500 46100 42500 4
N 44700 42400 44700 42500 4
N 43300 42500 43300 42400 4
N 42900 41500 44900 41500 4
C 44800 41500 1 90 0 resistor-1.sym
{
T 44400 41800 5 10 0 0 90 0 1
device=RESISTOR
T 44800 41500 5 10 0 0 90 0 1
footprint=0805
T 44500 41900 5 10 1 1 90 0 1
refdes=R22
T 45000 41900 5 10 1 1 90 0 1
value=1k
}
N 45000 42500 45000 43000 4
N 45000 43000 45400 43000 4
{
T 45500 42900 5 10 1 1 0 0 1
netname=VBIAS
}
N 45800 41500 46800 41500 4
N 46100 41500 46100 42100 4
N 47700 41500 48400 41500 4
N 48400 41500 48400 42300 4
N 48400 42300 49000 42300 4
C 49400 41400 1 0 0 gnd-1.sym
N 49000 41900 49000 40700 4
N 50200 40700 50200 42100 4
N 47900 40700 47700 40700 4
{
T 47100 40600 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 42100 52300 42100 4
{
T 52400 42000 5 10 1 1 0 0 1
netname=IDET4
}
C 51700 40900 1 0 0 gnd-1.sym
C 49300 42500 1 0 0 5V-plus-1.sym
C 46200 41400 1 0 1 res-pack4-1.sym
{
T 46200 41400 5 10 0 0 0 6 1
slot=3
T 45300 41200 5 10 1 1 0 6 1
value=10k
T 46000 41000 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 45700 41200 5 10 1 1 0 6 1
refdes=R11
}
C 46400 41400 1 0 0 res-pack4-1.sym
{
T 46400 41400 5 10 0 0 0 0 1
slot=4
T 47300 41200 5 10 1 1 0 0 1
value=10k
T 46600 41000 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 46900 41200 5 10 1 1 0 0 1
refdes=R11
}
C 46100 41900 1 0 0 tsv914-1.sym
{
T 46925 42050 5 8 0 0 0 0 1
device=TSV914
T 46100 41900 5 10 0 1 0 0 1
slot=4
T 46400 42200 5 10 1 1 0 0 1
refdes=U5
T 45400 42200 5 10 1 1 0 0 1
device=TSV914
T 46100 41900 5 10 0 0 0 0 1
footprint=SO14
}
C 49000 41700 1 0 0 tsv914-1.sym
{
T 49825 41850 5 8 0 0 0 0 1
device=TSV914
T 49300 42000 5 10 0 1 0 0 1
slot=3
T 49300 42000 5 10 1 1 0 0 1
refdes=U5
T 49000 41700 5 10 0 0 0 0 1
footprint=SO14
}
C 46500 41600 1 0 0 gnd-1.sym
N 50300 42100 50000 42100 4
N 48800 40700 49200 40700 4
C 47500 40600 1 0 0 res-pack2-1.sym
{
T 47500 40600 5 10 0 0 0 0 1
slot=2
T 47995 40400 5 10 1 1 0 0 1
refdes=R12
T 47700 40200 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 40400 5 10 1 1 0 0 1
value=1k
}
C 48800 40600 1 0 0 res-pack2-1.sym
{
T 48800 40600 5 10 0 0 0 0 1
slot=2
T 49195 40400 5 10 1 1 0 0 1
refdes=R13
T 49700 40400 5 10 1 1 0 0 1
value=100k
T 49100 40200 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 40700 50200 40700 4
C 42900 42400 1 270 0 mmbd4448dw-1.sym
{
T 43500 42000 5 10 0 0 270 0 1
device=MMBD4448DW
T 42898 42405 5 10 0 1 270 0 1
footprint=SOT363
T 43200 41300 5 10 1 1 0 0 1
refdes=D8
T 42900 42400 5 10 0 0 0 0 1
slot=1
}
C 43500 41500 1 270 1 mmbd4448dw-1.sym
{
T 44100 41900 5 10 0 0 90 2 1
device=MMBD4448DW
T 43498 41495 5 10 0 1 90 2 1
footprint=SOT363
T 43500 41500 5 10 0 0 180 6 1
slot=2
T 43800 41400 5 10 1 1 180 6 1
refdes=D8
}
N 43900 42400 43900 42500 4
C 47500 41900 1 0 0 mmbd4448dw-1.sym
{
T 47900 42500 5 10 0 0 0 0 1
device=MMBD4448DW
T 47495 41898 5 10 0 1 0 0 1
footprint=SOT363
T 47500 41900 5 10 0 1 0 0 1
slot=1
T 47900 42800 5 10 1 1 0 0 1
refdes=D6
}
N 47500 42300 47100 42300 4
C 49900 42000 1 0 0 res-pack4-1.sym
{
T 49900 42000 5 10 0 0 0 0 1
slot=4
T 50800 42600 5 10 1 1 0 0 1
value=10k
T 50200 42400 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 42600 5 10 1 1 0 0 1
refdes=R8
}
C 46400 42700 1 0 0 5V-plus-1.sym
C 61500 39300 1 0 0 gnd-1.sym
N 60900 39600 61600 39600 4
N 61600 39600 61600 40000 4
N 61100 40500 60300 40500 4
{
T 60000 40600 5 10 1 1 0 0 1
netname=ISET3
}
C 61400 41100 1 0 0 5V-plus-1.sym
N 61600 41100 61600 40900 4
C 64500 39300 1 0 0 gnd-1.sym
N 63900 39600 64600 39600 4
N 64600 39600 64600 40000 4
N 64100 40500 63300 40500 4
{
T 63000 40600 5 10 1 1 0 0 1
netname=ISET4
}
C 64400 41100 1 0 0 5V-plus-1.sym
N 64600 41100 64600 40900 4
C 55500 39300 1 0 0 gnd-1.sym
N 54900 39600 55600 39600 4
N 55600 39600 55600 40000 4
N 55100 40500 54300 40500 4
{
T 54000 40600 5 10 1 1 0 0 1
netname=ISET1
}
C 55400 41100 1 0 0 5V-plus-1.sym
N 55600 41100 55600 40900 4
C 58500 39300 1 0 0 gnd-1.sym
N 57900 39600 58600 39600 4
N 58600 39600 58600 40000 4
N 58100 40500 57300 40500 4
{
T 57000 40600 5 10 1 1 0 0 1
netname=ISET2
}
C 58400 41100 1 0 0 5V-plus-1.sym
N 58600 41100 58600 40900 4
N 62900 47200 64300 47200 4
{
T 64900 47100 5 10 1 1 0 6 1
netname=ISET1
}
N 62900 46900 64300 46900 4
{
T 64900 46800 5 10 1 1 0 6 1
netname=ISET2
}
N 62900 49900 64300 49900 4
{
T 64900 49800 5 10 1 1 0 6 1
netname=ISET3
}
N 62900 49600 64300 49600 4
{
T 64900 49500 5 10 1 1 0 6 1
netname=ISET4
}
C 59800 57100 1 0 0 5V-plus-1.sym
N 60300 52500 60300 53100 4
C 60100 53100 1 0 0 5V-plus-1.sym
U 57200 47300 57200 54000 10 1
U 57200 54000 67700 54000 10 1
U 67700 54000 68400 54000 10 0
U 68400 54000 68400 47100 10 -1
N 58100 51700 57400 51700 4
{
T 57600 51700 5 10 1 1 0 0 1
netname=IN1-1
}
C 57400 51700 1 90 0 busripper-1.sym
{
T 57000 51700 5 8 0 0 90 0 1
device=none
}
N 58100 51400 57400 51400 4
{
T 57600 51400 5 10 1 1 0 0 1
netname=IN1-2
}
C 57400 51400 1 90 0 busripper-1.sym
{
T 57000 51400 5 8 0 0 90 0 1
device=none
}
N 58100 51100 57400 51100 4
{
T 57600 51100 5 10 1 1 0 0 1
netname=IN1-3
}
C 57400 51100 1 90 0 busripper-1.sym
{
T 57000 51100 5 8 0 0 90 0 1
device=none
}
N 58100 50800 57400 50800 4
{
T 57600 50800 5 10 1 1 0 0 1
netname=IN1-4
}
C 57400 50800 1 90 0 busripper-1.sym
{
T 57000 50800 5 8 0 0 90 0 1
device=none
}
N 58100 50500 57400 50500 4
{
T 57600 50500 5 10 1 1 0 0 1
netname=IN2-1
}
C 57400 50500 1 90 0 busripper-1.sym
{
T 57000 50500 5 8 0 0 90 0 1
device=none
}
N 58100 50200 57400 50200 4
{
T 57600 50200 5 10 1 1 0 0 1
netname=IN2-2
}
C 57400 50200 1 90 0 busripper-1.sym
{
T 57000 50200 5 8 0 0 90 0 1
device=none
}
C 68600 51400 1 180 0 busripper-1.sym
{
T 68600 51000 5 8 0 0 180 0 1
device=none
}
C 68600 51100 1 180 0 busripper-1.sym
{
T 68600 50700 5 8 0 0 180 0 1
device=none
}
C 68600 50800 1 180 0 busripper-1.sym
{
T 68600 50400 5 8 0 0 180 0 1
device=none
}
N 70000 50500 68600 50500 4
{
T 68800 50500 5 10 1 1 0 0 1
netname=IN1-4
}
C 68600 50500 1 180 0 busripper-1.sym
{
T 68600 50100 5 8 0 0 180 0 1
device=none
}
N 68600 48400 70000 48400 4
{
T 68800 48400 5 10 1 1 0 0 1
netname=IN2-1
}
N 68600 48100 70000 48100 4
{
T 68800 48100 5 10 1 1 0 0 1
netname=IN2-2
}
N 68600 47800 70000 47800 4
{
T 68800 47800 5 10 1 1 0 0 1
netname=IN2-3
}
N 70000 47500 68600 47500 4
{
T 68800 47500 5 10 1 1 0 0 1
netname=IN2-4
}
C 68600 48400 1 180 0 busripper-1.sym
{
T 68600 48000 5 8 0 0 180 0 1
device=none
}
C 68600 48100 1 180 0 busripper-1.sym
{
T 68600 47700 5 8 0 0 180 0 1
device=none
}
C 68600 47800 1 180 0 busripper-1.sym
{
T 68600 47400 5 8 0 0 180 0 1
device=none
}
C 68600 47500 1 180 0 busripper-1.sym
{
T 68600 47100 5 8 0 0 180 0 1
device=none
}
N 58100 47800 57400 47800 4
{
T 57600 47800 5 10 1 1 0 0 1
netname=IN2-3
}
N 58100 47500 57400 47500 4
{
T 57600 47500 5 10 1 1 0 0 1
netname=IN2-4
}
C 57400 47800 1 90 0 busripper-1.sym
{
T 57000 47800 5 8 0 0 90 0 1
device=none
}
C 57400 47500 1 90 0 busripper-1.sym
{
T 57000 47500 5 8 0 0 90 0 1
device=none
}
N 57500 54200 57500 56600 4
{
T 57500 54500 5 10 1 1 90 0 1
netname=IN2-1
}
C 57500 54200 1 270 0 busripper-1.sym
{
T 57900 54200 5 8 0 0 270 0 1
device=none
}
N 57500 56600 58300 56600 4
N 57800 56200 57800 54200 4
{
T 57800 54500 5 10 1 1 90 0 1
netname=IN2-2
}
C 57800 54200 1 270 0 busripper-1.sym
{
T 58200 54200 5 8 0 0 270 0 1
device=none
}
N 57800 56200 58300 56200 4
N 60400 54200 60400 56200 4
{
T 60400 54600 5 10 1 1 90 0 1
netname=IN1-4
}
C 60400 54200 1 270 0 busripper-1.sym
{
T 60800 54200 5 8 0 0 270 0 1
device=none
}
N 60400 56200 59700 56200 4
N 58300 55800 58300 54800 4
N 58300 54800 58800 54800 4
{
T 58900 54700 5 10 1 1 0 0 1
netname=\_RESET\_
}
N 66700 46900 66000 46900 4
{
T 66200 47000 5 10 1 1 0 0 1
netname=\_RESET\_
}
C 65800 48100 1 0 0 5V-plus-1.sym
C 42000 40100 1 90 0 led-3.sym
{
T 42250 40350 5 10 1 1 90 0 1
device=RED
T 41450 40550 5 10 1 1 90 0 1
refdes=D13
T 42000 40100 5 10 0 0 0 0 1
footprint=0805
}
N 41800 40100 42200 40100 4
{
T 42300 40000 5 10 1 1 0 0 1
netname=LCOM
}
N 41800 41000 42200 41000 4
{
T 42300 40900 5 10 1 1 0 0 1
netname=LDRV2
}
C 42000 43700 1 90 0 led-3.sym
{
T 42000 43700 5 10 0 0 0 0 1
footprint=0805
T 42250 43950 5 10 1 1 90 0 1
device=RED
T 41450 44150 5 10 1 1 90 0 1
refdes=D12
}
N 41800 43700 42200 43700 4
{
T 42300 43600 5 10 1 1 0 0 1
netname=LCOM
}
N 41800 44600 42200 44600 4
{
T 42300 44500 5 10 1 1 0 0 1
netname=LDRV1
}
C 42000 47700 1 90 0 led-3.sym
{
T 42000 47700 5 10 0 0 0 0 1
footprint=0805
T 42250 47950 5 10 1 1 90 0 1
device=RED
T 41450 48150 5 10 1 1 90 0 1
refdes=D11
}
N 41800 47700 42200 47700 4
{
T 42300 47600 5 10 1 1 0 0 1
netname=LDRV2
}
N 41800 48600 42200 48600 4
{
T 42300 48500 5 10 1 1 0 0 1
netname=LCOM
}
C 42100 51400 1 90 0 led-3.sym
{
T 42100 51400 5 10 0 0 0 0 1
footprint=0805
T 42350 51650 5 10 1 1 90 0 1
device=RED
T 41550 51850 5 10 1 1 90 0 1
refdes=D10
}
N 41900 51400 42300 51400 4
{
T 42400 51300 5 10 1 1 0 0 1
netname=LDRV1
}
N 41900 52300 42300 52300 4
{
T 42400 52200 5 10 1 1 0 0 1
netname=LCOM
}
N 58100 46600 57700 46600 4
{
T 57600 46500 5 10 1 1 0 6 1
netname=LCOM
}
C 54800 47100 1 0 0 resistor-1.sym
{
T 55100 47500 5 10 0 0 0 0 1
device=RESISTOR
T 54800 47400 5 10 1 1 0 0 1
refdes=R19
T 55300 47400 5 10 1 1 0 0 1
value=330
T 54800 47100 5 10 0 0 0 0 1
footprint=0805
}
N 55700 47200 58100 47200 4
N 54800 47200 54400 47200 4
{
T 54300 47100 5 10 1 1 0 6 1
netname=LDRV1
}
N 55700 46900 58100 46900 4
C 54800 46800 1 0 0 resistor-1.sym
{
T 55100 47200 5 10 0 0 0 0 1
device=RESISTOR
T 54800 46600 5 10 1 1 0 0 1
refdes=R20
T 55300 46600 5 10 1 1 0 0 1
value=330
T 54800 46800 5 10 0 0 0 0 1
footprint=0805
}
N 54800 46900 54400 46900 4
{
T 54300 46800 5 10 1 1 0 6 1
netname=LDRV2
}
N 51600 58800 51600 58700 4
N 51600 59700 52600 59700 4
C 63800 53100 1 270 0 capacitor-1.sym
{
T 64500 52900 5 10 0 1 270 0 1
device=CAPACITOR
T 64100 52800 5 10 1 1 0 0 1
refdes=C13
T 64700 52900 5 10 0 0 270 0 1
symversion=0.1
T 64100 52300 5 10 1 1 0 0 1
value=0.1uF
T 63800 53100 5 10 0 0 0 0 1
footprint=0805
}
C 64600 53100 1 270 0 capacitor-1.sym
{
T 65300 52900 5 10 0 1 270 0 1
device=CAPACITOR
T 64900 52800 5 10 1 1 0 0 1
refdes=C14
T 65500 52900 5 10 0 0 270 0 1
symversion=0.1
T 64900 52300 5 10 1 1 0 0 1
value=0.1uF
T 64600 53100 5 10 0 0 0 0 1
footprint=0805
}
N 64000 52200 64800 52200 4
C 64700 51900 1 0 0 gnd-1.sym
C 46900 55000 1 270 0 capacitor-1.sym
{
T 47600 54800 5 10 0 1 270 0 1
device=CAPACITOR
T 47200 54700 5 10 1 1 0 0 1
refdes=C18
T 47800 54800 5 10 0 0 270 0 1
symversion=0.1
T 47200 54200 5 10 1 1 0 0 1
value=0.1uF
T 46900 55000 5 10 0 0 0 0 1
footprint=0805
}
N 46600 55200 46600 53900 4
N 46600 55000 47100 55000 4
C 47000 53800 1 0 0 gnd-1.sym
C 46400 47300 1 0 0 5V-plus-1.sym
C 46900 47300 1 270 0 capacitor-1.sym
{
T 47600 47100 5 10 0 1 270 0 1
device=CAPACITOR
T 47800 47100 5 10 0 0 270 0 1
symversion=0.1
T 46900 47300 5 10 0 0 0 0 1
footprint=0805
T 47200 47000 5 10 1 1 0 0 1
refdes=C4
T 47200 46500 5 10 1 1 0 0 1
value=0.1uF
}
N 46600 47300 46600 46300 4
N 46600 47300 47100 47300 4
C 47000 46100 1 0 0 gnd-1.sym
C 55100 39200 1 90 0 cap-pack4-1.sym
{
T 54605 40200 5 10 1 1 180 0 1
refdes=C11
T 55100 39200 5 10 0 0 0 0 1
slot=1
T 54100 39800 5 10 1 1 0 0 1
value=0.1uF
T 53100 39600 5 10 1 1 0 0 1
device=CL31B104KACNBNC
T 53400 39400 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 58100 39200 1 90 0 cap-pack4-1.sym
{
T 57605 40200 5 10 1 1 180 0 1
refdes=C11
T 58100 39200 5 10 0 0 0 0 1
slot=2
T 57100 39800 5 10 1 1 0 0 1
value=0.1uF
T 55900 39600 5 10 1 1 0 0 1
device=CL31B104KACNBNC
T 56400 39400 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 61100 39200 1 90 0 cap-pack4-1.sym
{
T 60605 40200 5 10 1 1 180 0 1
refdes=C11
T 61100 39200 5 10 0 0 0 0 1
slot=3
T 60100 39800 5 10 1 1 0 0 1
value=0.1uF
T 58900 39600 5 10 1 1 0 0 1
device=CL31B104KACNBNC
T 59400 39400 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 64100 39200 1 90 0 cap-pack4-1.sym
{
T 63605 40200 5 10 1 1 180 0 1
refdes=C11
T 64100 39200 5 10 0 0 0 0 1
slot=4
T 63100 39800 5 10 1 1 0 0 1
value=0.1uF
T 61800 39600 5 10 1 1 0 0 1
device=CL31B104KACNBNC
T 62400 39400 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 57800 59500 1 0 0 mcp1702-1.sym
{
T 59200 60500 5 10 1 1 0 6 1
refdes=U3
T 58700 60500 5 10 1 1 0 6 1
device=AP2120
T 58700 60500 5 10 0 1 0 0 1
footprint=SOT23
}
C 58500 58700 1 0 0 gnd-1.sym
C 59500 60100 1 270 0 capacitor-1.sym
{
T 60200 59900 5 10 0 1 270 0 1
device=CAPACITOR
T 59800 59800 5 10 1 1 0 0 1
refdes=C12
T 60400 59900 5 10 0 0 270 0 1
symversion=0.1
T 59800 59300 5 10 1 1 0 0 1
value=1uF
T 59500 60100 5 10 0 0 0 0 1
footprint=0805
}
N 59400 60100 61200 60100 4
{
T 61400 60000 5 10 1 1 0 0 1
netname=VBIAS
}
N 58600 59000 58600 59500 4
N 58600 59200 60700 59200 4
C 55700 40000 1 90 0 pot-1.sym
{
T 54800 40800 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 55300 40600 5 10 1 1 90 0 1
refdes=R14
T 54200 40800 5 10 0 0 90 0 1
footprint=bourns3266
T 55800 40300 5 10 1 1 0 0 1
value=100k
}
C 58700 40000 1 90 0 pot-1.sym
{
T 57800 40800 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 58300 40600 5 10 1 1 90 0 1
refdes=R15
T 57200 40800 5 10 0 0 90 0 1
footprint=bourns3266
T 58800 40300 5 10 1 1 0 0 1
value=100k
}
C 61700 40000 1 90 0 pot-1.sym
{
T 60800 40800 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 61300 40600 5 10 1 1 90 0 1
refdes=R16
T 60200 40800 5 10 0 0 90 0 1
footprint=bourns3266
T 61800 40300 5 10 1 1 0 0 1
value=100k
}
C 64700 40000 1 90 0 pot-1.sym
{
T 63800 40800 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 64300 40600 5 10 1 1 90 0 1
refdes=R17
T 63200 40800 5 10 0 0 90 0 1
footprint=bourns3266
T 64800 40300 5 10 1 1 0 0 1
value=100k
}
C 52000 52000 1 90 0 cap-pack4-1.sym
{
T 51505 53000 5 10 1 1 180 0 1
refdes=C3
T 52000 52000 5 10 0 0 0 0 1
slot=1
T 52100 52700 5 10 1 1 0 0 1
value=1uF
T 52100 52500 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 52300 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 52000 48400 1 90 0 cap-pack4-1.sym
{
T 51505 49400 5 10 1 1 180 0 1
refdes=C3
T 52000 48400 5 10 0 0 0 0 1
slot=2
T 52100 49100 5 10 1 1 0 0 1
value=1uF
T 52100 48900 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 48700 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 52000 44400 1 90 0 cap-pack4-1.sym
{
T 51505 45400 5 10 1 1 180 0 1
refdes=C3
T 52000 44400 5 10 0 0 0 0 1
slot=3
T 52100 45100 5 10 1 1 0 0 1
value=1uF
T 52100 44900 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 44700 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 52000 40800 1 90 0 cap-pack4-1.sym
{
T 51505 41800 5 10 1 1 180 0 1
refdes=C3
T 52000 40800 5 10 0 0 0 0 1
slot=4
T 52100 41500 5 10 1 1 0 0 1
value=1uF
T 52100 41300 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 41100 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 70800 53800 1 270 0 capacitor-1.sym
{
T 71500 53600 5 10 0 1 270 0 1
device=CAPACITOR
T 71100 53500 5 10 1 1 0 0 1
refdes=C19
T 71700 53600 5 10 0 0 270 0 1
symversion=0.1
T 71100 53000 5 10 1 1 0 0 1
value=10uF
T 70800 53800 5 10 0 0 0 0 1
footprint=0805
}
C 70900 52600 1 0 0 gnd-1.sym
C 70000 52300 1 90 0 mbr0540-1.sym
{
T 70328 52122 5 10 1 1 90 0 1
device=MBR0520LT1G
T 69500 52600 5 10 1 1 90 0 1
refdes=D14
T 69168 52641 5 10 0 1 90 0 1
footprint=SOD123
}
N 69800 51700 69800 52300 4
N 69800 53200 69800 54500 4
N 71000 53800 69800 53800 4
T 67900 55100 9 10 1 0 0 0 2
For MRB-BD42: Omit C19 and substitute 0-ohm 1206 for D14
For MRB-BD4X: Populate D14 and C19
C 60800 59200 1 90 0 resistor-1.sym
{
T 60400 59500 5 10 0 0 90 0 1
device=RESISTOR
T 61200 59800 5 10 1 1 180 0 1
refdes=R23
T 61200 59500 5 10 1 1 180 0 1
value=330
T 60800 59200 5 10 0 0 90 0 1
footprint=0805
}
T 62500 59600 9 10 1 0 0 0 2
Note: R23 prevents current pushed through the feedback networks of
 the op-amps from affecting the 1.2V reference rail
