# bw_diag.tcl — diagnose the on-board HyperBus test: read counters even on timeout,
# sweep small LEN, so we can tell write-phase vs read-phase failure. JTAG only, no re-fit.
set CTRL 0x00; set STATUS 0x00; set LEN 0x04; set BASE 0x08
set WRCYC 0x0C; set RDCYC 0x10; set ERRCNT 0x14; set MAGIC 0x1C
proc rd32 {m a} { return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}] }
set paths [get_service_paths master]
if {[llength $paths]==0} { puts "ERROR: no master service"; exit 1 }
set m [lindex $paths 0]; open_service master $m
puts [format "MAGIC = 0x%08X" [rd32 $m $MAGIC]]
foreach L {1 4 16 64} {
    master_write_32 $m $LEN $L
    master_write_32 $m $BASE 0x0
    master_write_32 $m $CTRL 0x1
    set st 0
    for {set i 0} {$i < 20000} {incr i} { set st [rd32 $m $STATUS]; if {($st & 0x2)!=0} break }
    set wr [rd32 $m $WRCYC]; set rd [rd32 $m $RDCYC]; set er [rd32 $m $ERRCNT]; set st [rd32 $m $STATUS]
    puts [format "LEN=%-3d  STATUS=0x%08X (busy=%d done=%d err=%d)  WR_CYC=%d  RD_CYC=%d  ERR_COUNT=%d" \
            $L $st [expr {$st&1}] [expr {($st>>1)&1}] [expr {($st>>2)&1}] $wr $rd $er]
}
close_service master $m
