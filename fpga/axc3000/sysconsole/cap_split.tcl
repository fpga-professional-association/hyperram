# cap_split.tcl — capture a SPLIT multi-burst run (LEN=32, BURST_WORDS=16) to inspect the
# write->write boundary + any commit-read transaction. Same capture buffer as cap_dump32.tcl.
set BW_CTRL   0x00
set BW_LEN    0x04
set BW_BASE   0x08
set BW_BURSTW 0x2C
set BW_MAGIC  0x1C
set CAP_CTRL  0x100
set CAP_RDADD 0x104
set CAP_LO    0x108
proc rd32 {m a} { return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}] }
set m [lindex [get_service_paths master] 0]
open_service master $m
puts [format "MAGIC = 0x%08X" [rd32 $m $BW_MAGIC]]
master_write_32 $m $BW_BURSTW 16
master_write_32 $m $BW_LEN  32
master_write_32 $m $BW_BASE 0x0
master_write_32 $m $CAP_CTRL 0x1
master_write_32 $m $BW_CTRL 0x1
set st 0
for {set i 0} {$i < 20000} {incr i} { set st [rd32 $m $CAP_CTRL]; if {($st & 0x2)!=0} break }
set fill [expr {($st >> 16) & 0xffff}]
puts [format "CAPTURE DONE: fill=%d" $fill]
for {set i 0} {$i < $fill} {incr i} {
    master_write_32 $m $CAP_RDADD $i
    set lo [expr {[lindex [master_read_32 $m $CAP_LO 1] 0] & 0xffffffff}]
    puts [format "idx=%d HI=0x00000000 LO=0x%08X" $i $lo]
}
close_service master $m
