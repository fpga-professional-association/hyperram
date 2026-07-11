# cap_dumponly.tcl — TEMP (issue #13 forensics): dump a COMPLETED capture. No arm, no trigger.
set CAP_CTRL  0x100
set CAP_RDADD 0x104
set CAP_LO    0x108
proc rd32 {m a} { return [expr {[lindex [master_read_32 $m $a 1] 0] & 0xffffffff}] }
set paths [get_service_paths master]
if {[llength $paths] == 0} { puts "ERROR: no master service"; exit 1 }
set m [lindex $paths 0]
open_service master $m
set st [rd32 $m $CAP_CTRL]
set fill [expr {($st >> 16) & 0xffff}]
if {(($st >> 1) & 1) == 0} {
    puts [format "ERROR: capture not done. STATUS=0x%08X (armed=%d fill=%d)" $st [expr {$st & 1}] $fill]
    close_service master $m
    exit 1
}
puts [format "CAPTURE DONE: fill=%d" $fill]
for {set i 0} {$i < $fill} {incr i} {
    master_write_32 $m $CAP_RDADD $i
    set pair [master_read_32 $m $CAP_LO 2]
    puts [format "S %d %08X %08X" $i [expr {[lindex $pair 1] & 0xffffffff}] [expr {[lindex $pair 0] & 0xffffffff}]]
}
puts "DUMP COMPLETE"
close_service master $m
