# bw_read.tcl — run the on-chip HyperBus bandwidth test over JTAG and print MB/s.
#
# Control plane ONLY over JTAG (PLAN §8 method E): this programs LEN/BASE_ADDR, pulses CTRL.start,
# polls STATUS.done, and reads the on-chip cycle/error counters. The measured WR_CYCLES/RD_CYCLES
# cover the on-chip Avalon datapath, NOT the JTAG access time, so the reported MB/s is the real
# HyperBus throughput.
#
# Run inside Quartus System Console (headless):
#   system-console --script=sysconsole/bw_read.tcl
# optionally with args:  --script=sysconsole/bw_read.tcl <LEN_words> <BASE_word_addr_hex>
#
# CSR map (docs/BW_TEST.md). The JTAG-to-Avalon master is BYTE addressed; register k is at byte 4*k:
#   0x00 CTRL(w)/STATUS(r)   0x04 LEN   0x08 BASE_ADDR   0x0C WR_CYCLES   0x10 RD_CYCLES
#   0x14 ERR_COUNT           0x18 DATA_BYTES_PER_WORD    0x1C VERSION/MAGIC ("HBWT"=0x48425754)

# ---- configuration -------------------------------------------------------
set F_CLK      50.0e6       ;# IOPLL clk (outclk0) — the on-chip word clock, MUST match make_bw_sys.tcl
set LEN_WORDS  4096         ;# words per phase (override: arg 1)
set BASE_ADDR  0x00000000   ;# starting WORD address, MSB=0 => memory space (override: arg 2)

set BURSTW    0            ;# HyperBus burst length (words); 0 => leave at bitstream default (override: arg 3)
if {$argc >= 1} { set LEN_WORDS [lindex $argv 0] }
if {$argc >= 2} { set BASE_ADDR [lindex $argv 1] }
if {$argc >= 3} { set BURSTW    [lindex $argv 2] }
if {$argc >= 4} { set F_CLK [expr {double([lindex $argv 3]) * 1.0e6}] }   ;# arg 4 = clk (word) freq in MHz
set CALV -1                ;# arg 5 = REG_CAL image (live read-eye cal, issue #10: [0]=capture_phase
if {$argc >= 5} { set CALV [lindex $argv 4] }   ;# [3:1]=preamble_skip [8:4]=rx_tap [9]=pair_skew); -1 => POR seed

# CSR byte offsets
set CTRL   0x00
set STATUS 0x00
set LEN    0x04
set BASE   0x08
set WRCYC  0x0C
set RDCYC  0x10
set ERRCNT 0x14
set BYTES  0x18
set MAGIC  0x1C
set ERRADDR 0x20   ;# first-mismatch WORD address
set ERRGOT  0x24   ;# first-mismatch value returned
set ERREXP  0x28   ;# first-mismatch value expected
set BURSTWR 0x2C   ;# R/W HyperBus burst length (words)
set CALR    0x34   ;# R/W live PHY read-eye calibration image (issue #10 REG_CAL)

# ---- helpers -------------------------------------------------------------
proc rd32 {m a} {
    # master_read_32 returns a list of one 32-bit value; normalise to an unsigned integer.
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

# ---- open the JTAG-to-Avalon master service ------------------------------
set paths [get_service_paths master]
if {[llength $paths] == 0} {
    puts "ERROR: no Avalon 'master' service found. Is the board programmed and USB-Blaster III attached?"
    exit 1
}
if {[llength $paths] > 1} {
    puts "NOTE: multiple master services found; using the first:"
    foreach p $paths { puts "   $p" }
}
set m [lindex $paths 0]
open_service master $m
puts "Opened master service: $m"

# ---- sanity: MAGIC ("HBWT") ----------------------------------------------
set magic [rd32 $m $MAGIC]
puts [format "VERSION/MAGIC = 0x%08X (expect 0x48425754 \"HBWT\")" $magic]
if {$magic != 0x48425754} {
    puts "WARNING: unexpected MAGIC — wrong bitstream, address map, or JTAG target."
}
set bpw [rd32 $m $BYTES]
if {$bpw == 0} { set bpw 2 }   ;# DATA_BYTES_PER_WORD, constant 2

# ---- program the run -----------------------------------------------------
puts [format "Programming LEN=%d words, BASE_ADDR=0x%08X ..." $LEN_WORDS $BASE_ADDR]
master_write_32 $m $LEN  $LEN_WORDS
master_write_32 $m $BASE $BASE_ADDR
if {$BURSTW != 0} { master_write_32 $m $BURSTWR $BURSTW }
if {$BURSTW != 0} { master_write_32 $m 0x30 $BURSTW }  ;# REG_RBURSTW (issue #2): read burst = write burst
if {$CALV != -1} {
    master_write_32 $m $CALR $CALV
    puts [format "REG_CAL       = 0x%08X (live read-eye cal)" [rd32 $m $CALR]]
}
set burstw_now [rd32 $m $BURSTWR]
puts [format "BURST_WORDS   = %d (HyperBus burst length)" $burstw_now]

# ---- pulse CTRL.start (self-clearing strobe) -----------------------------
master_write_32 $m $CTRL 0x1

# ---- poll STATUS.done (bit1), with a timeout -----------------------------
set done 0
for {set i 0} {$i < 100000} {incr i} {
    set st [rd32 $m $STATUS]
    if {($st & 0x2) != 0} { set done 1; break }   ;# STATUS.done
}
if {!$done} {
    puts "ERROR: test did not complete (STATUS.done never asserted). Last STATUS = [format 0x%08X $st]"
    close_service master $m
    exit 1
}

# ---- read back the counters ----------------------------------------------
set wr_cycles [rd32 $m $WRCYC]
set rd_cycles [rd32 $m $RDCYC]
set err_count [rd32 $m $ERRCNT]
set status    [rd32 $m $STATUS]

# ---- bandwidth: MB/s = LEN * bytes_per_word * f_clk / (cycles * 1e6) -----
proc mbps {len bpw fclk cycles} {
    if {$cycles == 0} { return 0.0 }
    return [expr {double($len) * double($bpw) * $fclk / (double($cycles) * 1.0e6)}]
}
set wr_mbps [mbps $LEN_WORDS $bpw $F_CLK $wr_cycles]
set rd_mbps [mbps $LEN_WORDS $bpw $F_CLK $rd_cycles]

puts "---------------------------------------------------------------"
puts [format "STATUS        = 0x%08X (busy=%d done=%d error=%d)" \
        $status [expr {$status & 1}] [expr {($status>>1)&1}] [expr {($status>>2)&1}]]
puts [format "f_clk         = %.3f MHz" [expr {$F_CLK/1e6}]]
puts [format "LEN           = %d words   (%d bytes/phase, %d bytes/word)" \
        $LEN_WORDS [expr {$LEN_WORDS*$bpw}] $bpw]
puts [format "WR_CYCLES     = %d   -> WRITE %.2f MB/s" $wr_cycles $wr_mbps]
puts [format "RD_CYCLES     = %d   -> READ  %.2f MB/s" $rd_cycles $rd_mbps]
puts [format "ERR_COUNT     = %d" $err_count]
if {$err_count == 0 && (($status>>2)&1) == 0} {
    puts "RESULT        = PASS (data integrity OK)"
} else {
    set eaddr [rd32 $m $ERRADDR]
    set egot  [rd32 $m $ERRGOT]
    set eexp  [rd32 $m $ERREXP]
    puts [format "FIRST ERROR   = word\[0x%08X\]  got=0x%04X  exp=0x%04X  (word %d in burst of 16)" \
            $eaddr [expr {$egot & 0xffff}] [expr {$eexp & 0xffff}] [expr {$eaddr % 16}]]
    puts "RESULT        = FAIL (read mismatches — see FIRST ERROR above)"
}
puts "---------------------------------------------------------------"

close_service master $m
