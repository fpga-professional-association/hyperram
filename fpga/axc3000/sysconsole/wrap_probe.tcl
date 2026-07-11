# wrap_probe.tcl <B_word_hex> — L-F wrapped-write repair probe (issue #13).
#
# Arms ONE wrapped write (cmd_wrap=1) of WRAP_PROBE_WORDS (=16, the CR0 0x8F1F 32B group) at word
# address B via REG_WRAP (byte 0x44), waits for it to complete, then runs a READ-ONLY probe over
# [B-8, B+16) (CTRL bit1 — write phase skipped) and dumps the wound-map FIFO. This tests (a) whether
# a wrapped write itself wounds, and (b) whether a wrapped write repairs a linear-write wound zone.
#
# Set the data pattern first (pat_set.tcl) if you want a non-gen background; REG_PAT applies to both
# the wrapped write and the RO scoreboard, so they agree. B must be nonzero (0 = "no probe armed").
#
# Reuses the bw_read.tcl RO-run idiom + the emap_dump.tcl pop protocol inline (one JTAG session).
# Run inside Quartus System Console (headless); caller holds the board lock:
#   flock -w 600 /tmp/axc3000-devkit.lock system-console --script=sysconsole/wrap_probe.tcl <B_word_hex>

# ---- CSR byte offsets ----------------------------------------------------
set CTRL           0x00
set STATUS         0x00
set LEN            0x04
set BASE           0x08
set ERRCNT         0x14
set MAGIC          0x1C
set REG_EMAP_STAT  0x3C
set REG_WRAP       0x44
set REG_EMAP_IDX   0x48
set REG_EMAP_ADDR  0x4C
set REG_EMAP_DATA  0x50

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}
proc poll_done {m status} {
    for {set i 0} {$i < 100000} {incr i} {
        set st [rd32 $m $status]
        if {($st & 0x2) != 0} { return $st }   ;# STATUS.done
    }
    return -1
}

# ---- args ----------------------------------------------------------------
if {$argc < 1} {
    puts "USAGE: wrap_probe.tcl <B_word_hex>   (nonzero target WORD address for the wrapped write)"
    exit 1
}
set B [expr {int([lindex $argv 0])}]
if {$B == 0} { puts "ERROR: B must be nonzero (REG_WRAP=0 arms nothing)."; exit 1 }

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

# ---- arm the wrapped write (REG_WRAP = B) --------------------------------
set st0 [rd32 $m $STATUS]
if {($st0 & 0x1) != 0} { puts "WARNING: STATUS.busy=1 before arming — REG_WRAP is ignored while busy." }
puts [format "Arming wrapped write: REG_WRAP = 0x%08X (16 words, cmd_wrap=1) ..." $B]
master_write_32 $m $REG_WRAP $B
set st [poll_done $m $STATUS]
if {$st < 0} {
    puts "ERROR: wrapped write never completed (STATUS.done never asserted)."
    close_service master $m
    exit 1
}
puts [format "Wrapped write DONE: STATUS = 0x%08X (busy=%d done=%d error=%d)" \
        $st [expr {$st & 1}] [expr {($st>>1)&1}] [expr {($st>>2)&1}]]

# ---- READ-ONLY probe over [B-8, B+16) ------------------------------------
set pbase [expr {$B >= 8 ? $B - 8 : 0}]
set plen  [expr {($B + 16) - $pbase}]
puts [format "RO probe: LEN=%d words, BASE=0x%08X (covers \[0x%08X, 0x%08X)) ..." \
        $plen $pbase $pbase [expr {$B + 16}]]
master_write_32 $m $LEN  $plen
master_write_32 $m $BASE $pbase
master_write_32 $m $CTRL 0x3          ;# start + READ-ONLY (skip write phase)
set st [poll_done $m $STATUS]
if {$st < 0} {
    puts "ERROR: RO probe never completed (STATUS.done never asserted)."
    close_service master $m
    exit 1
}
set err [rd32 $m $ERRCNT]
puts [format "RO probe DONE: STATUS = 0x%08X (error=%d)  ERR_COUNT = %d" $st [expr {($st>>2)&1}] $err]
if {$err == 0} {
    puts "RESULT = ERR_COUNT=0 over the probe window (wrapped write did NOT wound / repaired the zone)."
} else {
    puts "RESULT = ERR_COUNT>0 — see the wound map below."
}

# ---- dump the wound-map FIFO (inline emap_dump; separate JTAG transactions per A5) --------------
set stat  [rd32 $m $REG_EMAP_STAT]
set count [expr {$stat & 0x7F}]
set ov    [expr {($stat >> 8) & 1}]
puts [format "EMAP: count=%d overflow=%d" $count $ov]
if {$ov} { puts "WARNING: EMAP overflowed (>64) — first 64 (nearest BASE) retained." }
if {$count > 0} {
    puts "  idx  word-addr     got     exp"
    for {set i 0} {$i < $count} {incr i} {
        master_write_32 $m $REG_EMAP_IDX $i
        set addr [rd32 $m $REG_EMAP_ADDR]
        set data [rd32 $m $REG_EMAP_DATA]
        puts [format "  %3d  0x%08X  0x%04X  0x%04X" \
                $i $addr [expr {($data >> 16) & 0xffff}] [expr {$data & 0xffff}]]
    }
}

close_service master $m
