# cap_cfg.tcl <N> — set the capture trigger to the Nth hb_cs_n FALLING edge (REG_CAPCFG, byte 0x110,
# issue #13), then chain to cap_dump.tcl to arm + trigger + dump. Aim the capture at the chop reopen
# / row-end close (2nd, 3rd, ... CS# open) instead of always the first.
#
# N is 1-based; N=1 reproduces the legacy first-edge trigger (the POR default). REG_CAPCFG is a plain
# register that persists across JTAG sessions, so we poke it, close, and let cap_dump.tcl re-open and
# run the capture — the latched N takes effect at the next arm.
#
# Run inside Quartus System Console (headless); caller holds the board lock:
#   flock -w 600 /tmp/axc3000-devkit.lock system-console --script=sysconsole/cap_cfg.tcl <N>

# ---- CSR byte offsets ----------------------------------------------------
set MAGIC        0x1C
set REG_CAPCFG   0x110    ;# capture CSR word 4 (base 0x100 + word*4)

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

# ---- args ----------------------------------------------------------------
set N 1
if {$argc >= 1} { set N [expr {int([lindex $argv 0])}] }
if {$N < 1} { set N 1 }

# ---- open, gate, poke REG_CAPCFG, close ----------------------------------
set paths [get_service_paths master]
if {[llength $paths] == 0} {
    puts "ERROR: no Avalon 'master' service found. Is the board programmed and USB-Blaster III attached?"
    exit 1
}
set m [lindex $paths 0]
open_service master $m
puts "Opened master service: $m"

set magic [rd32 $m $MAGIC]
if {$magic != 0x48425755} {
    puts [format "ERROR: MAGIC = 0x%08X, expected 0x48425755 (\"HBWU\" instrumented). Wrong bitstream." $magic]
    close_service master $m
    exit 1
}

master_write_32 $m $REG_CAPCFG $N
puts [format "REG_CAPCFG = %d (trigger on the %d-th hb_cs_n falling edge after arming)" [rd32 $m $REG_CAPCFG] $N]
close_service master $m

# ---- chain to cap_dump.tcl (re-opens the service, arms, triggers, dumps) --
# REG_CAPCFG persists across the close/re-open, so the latched N applies to cap_dump's arm.
set here [file dirname [file normalize [info script]]]
puts "Chaining to cap_dump.tcl ..."
source [file join $here cap_dump.tcl]
