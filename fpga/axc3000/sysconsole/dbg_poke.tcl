# dbg_poke.tcl <field> <value> — named-knob read-modify-write of REG_DBG (byte 0x38) on the
# issue-#13 instrumented AXC3000 build. Fields map onto the REG_DBG bit layout (spec §1):
#   wrtrim [3:0]   lat [7:4]   prewin [9]   pn [12:10]   marker [13]   posthold [14]   ckstretchoff [15]
#   contig [16]    endcw [17]      (round 3: A heal contiguous ST_IDLE write reopens; B end-of-row commit-WRITE
#                                   — a masked 4-word write at the row-aligned end; the prewin tail heals the
#                                   orphan home. `endcread` is a DEPRECATED alias of `endcw` — the round-2
#                                   end-READ was falsified (a read sprays the orphan one row low, never home).)
# Special field `reprog`: writes REG_DBG with bit8=1 — the CR0-reprogram strobe (self-clearing, reads
# back 0). To change latency: `dbg_poke lat 7` FIRST (sets the live seed), THEN `dbg_poke reprog`, and
# ONLY while STATUS.busy=0 — the pulse is LOST if the controller is busy (host responsibility, spec §1).
#
# Board POR REG_DBG = 0x00000063 (dbg_lat_clocks=6, dbg_wr_lat_trim=3 — bit-identical to this build's
# ctrl LATENCY_CLOCKS(6) / WR_LAT_TRIM(3), so every knob is legacy at reset).
#
# Run inside Quartus System Console (headless); the CALLER holds the board lock:
#   flock -w 600 /tmp/axc3000-devkit.lock system-console --script=sysconsole/dbg_poke.tcl <field> <value>
# (jtagconfig must have been run before system-console in the same privileged container — see README.)

# ---- CSR byte offsets ----------------------------------------------------
set STATUS  0x00
set MAGIC   0x1C
set REG_DBG 0x38

# field -> {lsb width}  (bit8 is the CR0-reprog strobe, handled separately as `reprog`)
array set FMAP {
    wrtrim       {0 4}
    lat          {4 4}
    prewin       {9 1}
    pn           {10 3}
    marker       {13 1}
    posthold     {14 1}
    ckstretchoff {15 1}
    contig       {16 1}
    endcw        {17 1}
    endcread     {17 1}
}

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

# ---- args ----------------------------------------------------------------
if {$argc < 1} {
    puts "USAGE: dbg_poke.tcl <field> <value>   fields: [lsort [array names ::FMAP]] reprog"
    exit 1
}
set field [lindex $argv 0]
if {$field eq "endcread"} {
    puts "NOTE: 'endcread' is a DEPRECATED alias of 'endcw' (bit17). Round 2's end-READ was falsified —"
    puts "      bit17 now self-issues an end-commit-WRITE. Same bit; use 'endcw' going forward."
    set field endcw
}
set value 0
if {$argc >= 2} { set value [expr {int([lindex $argv 1])}] }

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

# ---- read-modify-write ---------------------------------------------------
set cur [rd32 $m $REG_DBG]
if {$field eq "reprog"} {
    # Fire the CR0-reprogram strobe: re-write the current image with bit8=1 (self-clearing; reads 0).
    # The ctrl consumes it only when idle & init_done, so poke `lat` first and keep STATUS.busy=0.
    set st [rd32 $m $STATUS]
    if {($st & 0x1) != 0} { puts "WARNING: STATUS.busy=1 — CR0-reprog pulse may be lost (poke while busy=0)." }
    master_write_32 $m $REG_DBG [expr {($cur & 0xFFFFFEFF) | 0x100}]
    puts "REG_DBG CR0-reprog strobe fired (bit8=1, self-clearing)."
} else {
    if {![info exists FMAP($field)]} {
        puts "ERROR: unknown field '$field'. Valid: [lsort [array names FMAP]] reprog"
        close_service master $m
        exit 1
    }
    lassign $FMAP($field) lsb width
    set mask [expr {((1 << $width) - 1) << $lsb}]
    # Force bit8 (the strobe) low on every non-reprog write so a poke never accidentally re-runs CR0.
    set nv [expr {(($cur & ~$mask) | (($value << $lsb) & $mask)) & 0xFFFFFEFF}]
    master_write_32 $m $REG_DBG $nv
    puts [format "REG_DBG %s = %d  (0x%08X -> 0x%08X)" $field $value $cur $nv]
}

# ---- read back (bit8 always reads 0) -------------------------------------
set now [rd32 $m $REG_DBG]
puts [format "REG_DBG = 0x%08X  (wrtrim=%d lat=%d prewin=%d pn=%d marker=%d posthold=%d ckstretchoff=%d contig=%d endcw=%d)" \
        $now [expr {$now & 0xF}] [expr {($now >> 4) & 0xF}] [expr {($now >> 9) & 1}] \
        [expr {($now >> 10) & 0x7}] [expr {($now >> 13) & 1}] [expr {($now >> 14) & 1}] [expr {($now >> 15) & 1}] \
        [expr {($now >> 16) & 1}] [expr {($now >> 17) & 1}]]

close_service master $m
