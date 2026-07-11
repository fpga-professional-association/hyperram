# pat_set.tcl <0|1|2|3> — select the bw_test data pattern via REG_PAT (byte 0x40, issue #13).
#
# Applied to BOTH the write launch and the read expectation, so write and read always agree:
#   0 = gen_pattern(addr) (today)   1 = 0xFFFF   2 = 0x0000   3 = addr-echo (data = low 16 bits of addr)
# gen(0)=0 pitfall: with pat 0 the background at low addresses is 0, indistinguishable from a zeroed
# wound — use pat 1 or 3 (or BASE!=0) for any pass/attribution assertion (spec §1).
#
# Run inside Quartus System Console (headless); caller holds the board lock:
#   flock -w 600 /tmp/axc3000-devkit.lock system-console --script=sysconsole/pat_set.tcl <0|1|2|3>

# ---- CSR byte offsets ----------------------------------------------------
set MAGIC   0x1C
set REG_PAT 0x40

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

# ---- args ----------------------------------------------------------------
if {$argc < 1} {
    puts "USAGE: pat_set.tcl <0|1|2|3>   (0=gen 1=0xFFFF 2=0x0000 3=addr-echo)"
    exit 1
}
set p [expr {int([lindex $argv 0]) & 0x3}]

# ---- open the JTAG-to-Avalon master service ------------------------------
set paths [get_service_paths master]
if {[llength $paths] == 0} {
    puts "ERROR: no Avalon 'master' service found. Is the board programmed and USB-Blaster III attached?"
    exit 1
}
set m [lindex $paths 0]
open_service master $m
puts "Opened master service: $m"

# ---- gate on the instrumented build (MAGIC "HBWU" = 0x48425755) ----------
set magic [rd32 $m $MAGIC]
if {$magic != 0x48425755} {
    puts [format "ERROR: MAGIC = 0x%08X, expected 0x48425755 (\"HBWU\" instrumented). Wrong bitstream." $magic]
    close_service master $m
    exit 1
}

# ---- poke + read back ----------------------------------------------------
master_write_32 $m $REG_PAT $p
set now [rd32 $m $REG_PAT]
set names {gen(addr) 0xFFFF 0x0000 addr-echo}
puts [format "REG_PAT = %d (%s)" [expr {$now & 0x3}] [lindex $names [expr {$now & 0x3}]]]

close_service master $m
