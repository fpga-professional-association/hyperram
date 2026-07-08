# cap_dump.tcl — arm the on-chip HyperBus capture buffer (hyperbus_capture.sv, CSR base 0x100),
# trigger one small bw_test HyperRAM transaction (LEN=4 words @ BASE=0), and dump the raw 100 MHz
# samples over the JTAG-Avalon control plane.
#
# Run inside Quartus System Console (headless):
#   system-console --script=sysconsole/cap_dump.tcl
#
# CSR maps (byte offsets on the JTAG-to-Avalon master, decode bit m_address[8] — see top.sv):
#   hyperram_bw_test  @ 0x000:  0x00 CTRL/STATUS  0x04 LEN  0x08 BASE_ADDR ... 0x1C MAGIC ("HBWT")
#   hyperbus_capture  @ 0x100:  0x100 CTRL(w bit0=arm)/STATUS(r bit0=armed, bit1=done,
#                               bits[31:16]=fill)  0x104 RDADDR  0x108 DATA_LO  0x10C DATA_HI

# ---- CSR byte offsets ------------------------------------------------------
set BW_CTRL   0x00
set BW_LEN    0x04
set BW_BASE   0x08
set BW_MAGIC  0x1C

set CAP_CTRL  0x100      ;# w: bit0 = arm            r: STATUS
set CAP_RDADD 0x104      ;# w: sample index
set CAP_LO    0x108      ;# r: sample[31:0]
set CAP_HI    0x10C      ;# r: sample[63:32] (pad, reads 0)

# ---- helpers ---------------------------------------------------------------
proc rd32 {m a} {
    return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}]
}

# ---- open the JTAG-to-Avalon master service --------------------------------
set paths [get_service_paths master]
if {[llength $paths] == 0} {
    puts "ERROR: no Avalon 'master' service found. Is the board programmed and USB-Blaster III attached?"
    exit 1
}
set m [lindex $paths 0]
open_service master $m
puts "Opened master service: $m"

# ---- sanity: bw_test MAGIC --------------------------------------------------
set magic [rd32 $m $BW_MAGIC]
puts [format "BW MAGIC      = 0x%08X (expect 0x48425754 \"HBWT\")" $magic]
if {$magic != 0x48425754} {
    puts "WARNING: unexpected MAGIC — wrong bitstream, address map, or JTAG target."
}

# ---- ARM the capture (records from the first hb_cs_n low) -------------------
master_write_32 $m $CAP_CTRL 0x1
set st [rd32 $m $CAP_CTRL]
puts [format "CAP after arm = 0x%08X (armed=%d done=%d fill=%d)" \
        $st [expr {$st & 1}] [expr {($st >> 1) & 1}] [expr {($st >> 16) & 0xffff}]]

# ---- trigger one small HyperRAM transaction: LEN=4 words @ BASE=0 -----------
master_write_32 $m $BW_LEN  4
master_write_32 $m $BW_BASE 0x0
master_write_32 $m $BW_CTRL 0x1
puts "Triggered bw_test run: LEN=4 words, BASE_ADDR=0x00000000"

# ---- poll capture STATUS.done, with a timeout --------------------------------
set done 0
set st 0
for {set i 0} {$i < 20000} {incr i} {
    set st [rd32 $m $CAP_CTRL]
    if {($st & 0x2) != 0} { set done 1; break }
}
set fill [expr {($st >> 16) & 0xffff}]
if {!$done} {
    puts [format "ERROR: capture never completed. CAP STATUS = 0x%08X (armed=%d done=%d fill=%d)" \
            $st [expr {$st & 1}] [expr {($st >> 1) & 1}] $fill]
    puts "       (armed=1 fill=0 => hb_cs_n never went low: no HyperBus transaction was started)"
    close_service master $m
    exit 1
}
puts [format "CAPTURE DONE: fill = %d samples @ 100 MHz clk2x (10 ns/sample)" $fill]

# ---- sample word bit-field key (decode header) --------------------------------
puts "FIELD KEY (per sample, HI:LO = sample\[63:0\]):"
puts "  LO\[0\]=hb_cs_n  LO\[1\]=hb_ck  LO\[2\]=hb_dq_oe  LO\[10:3\]=hb_dq_o\[7:0\]  LO\[18:11\]=hb_dq_i\[7:0\]"
puts "  LO\[19\]=hb_rwds_oe  LO\[20\]=hb_rwds_o  LO\[21\]=hb_rwds_i"
puts "  LO\[22\]=av_read  LO\[23\]=av_write  LO\[24\]=av_waitrequest  LO\[25\]=av_readdatavalid"
puts "  LO\[31:26\]=0  HI\[31:0\]=0 (pad)"

# ---- dump every sample --------------------------------------------------------
for {set i 0} {$i < $fill} {incr i} {
    master_write_32 $m $CAP_RDADD $i
    set pair [master_read_32 $m $CAP_LO 2]      ;# one burst: DATA_LO (0x108) + DATA_HI (0x10C)
    set lo [expr {[lindex $pair 0] & 0xffffffff}]
    set hi [expr {[lindex $pair 1] & 0xffffffff}]
    puts [format "idx=%d HI=0x%08X LO=0x%08X" $i $hi $lo]
}
puts [format "DUMP COMPLETE: %d samples" $fill]

close_service master $m
