# emap_dump.tcl — dump the wound-map FIFO (REG_EMAP_*, issue #13) after a bw_test run.
#
# Every read-back mismatch of the LAST run is captured as {addr, got, exp} into a 64-deep RAM,
# independent of the legacy single-shot FIRST ERROR (REG_ERRADDR/GOT/EXP). This decodes an entire
# multi-wound pattern in one shot instead of guessing the 2nd..Nth windows.
#
# Pop protocol (spec §1, amendment A5): the RAM read is REGISTERED — write REG_EMAP_IDX with ONE JTAG
# transaction, then read REG_EMAP_ADDR/DATA with LATER ones. In System Console every master_* call is
# already a separate JTAG transaction (many clk apart), so the 1-cycle RAM read latency is invisible.
#
# Run AFTER a bw_read.tcl / wrap_probe.tcl run (the FIFO holds that run's wounds until the next
# start_stroke clears it). Inside Quartus System Console (headless); caller holds the board lock:
#   flock -w 600 /tmp/axc3000-devkit.lock system-console --script=sysconsole/emap_dump.tcl

# ---- CSR byte offsets ----------------------------------------------------
set MAGIC          0x1C
set REG_EMAP_STAT  0x3C     ;# r: {23'b0, ov[8], valid[7], count[6:0]}
set REG_EMAP_IDX   0x48     ;# w: read index [5:0] into emap_mem
set REG_EMAP_ADDR  0x4C     ;# r: emap_mem[IDX].addr (32-bit WORD address)
set REG_EMAP_DATA  0x50     ;# r: {got[31:16], exp[15:0]}

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

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

# ---- read STAT -> count / valid / overflow -------------------------------
set stat  [rd32 $m $REG_EMAP_STAT]
set count [expr {$stat & 0x7F}]
set valid [expr {($stat >> 7) & 1}]
set ov    [expr {($stat >> 8) & 1}]
puts [format "EMAP STAT = 0x%08X  count=%d  valid=%d  overflow=%d" $stat $count $valid $ov]
if {$ov} {
    puts "WARNING: EMAP overflowed (>64 wounds this run) — only the first 64 (nearest BASE) retained."
}
if {$count == 0} {
    puts "EMAP empty: no read-back mismatches in the last run (ERR_COUNT=0)."
    close_service master $m
    exit 0
}

# ---- walk IDX 0..count-1 (IDX write and ADDR/DATA reads are separate JTAG transactions, A5) --
puts "  idx  word-addr     got     exp"
for {set i 0} {$i < $count} {incr i} {
    master_write_32 $m $REG_EMAP_IDX $i           ;# transaction 1: set the read index
    set addr [rd32 $m $REG_EMAP_ADDR]             ;# transaction 2: registered ADDR slice
    set data [rd32 $m $REG_EMAP_DATA]             ;# transaction 3: registered DATA slice
    set got  [expr {($data >> 16) & 0xffff}]
    set exp  [expr {$data & 0xffff}]
    puts [format "  %3d  0x%08X  0x%04X  0x%04X" $i $addr $got $exp]
}
puts [format "DUMP COMPLETE: %d wound entr%s" $count [expr {$count == 1 ? "y" : "ies"}]]

close_service master $m
