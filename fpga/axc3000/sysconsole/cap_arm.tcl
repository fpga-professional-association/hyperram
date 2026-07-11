# cap_arm.tcl <N> — poke REG_CAPCFG=N and ARM the capture, then exit
# WITHOUT triggering a run. Pair with an external bw_read.tcl run and cap_dumponly.tcl.
set CAP_CTRL   0x100
set REG_CAPCFG 0x110
set N 1
if {$argc >= 1} { set N [expr {int([lindex $argv 0])}] }
set paths [get_service_paths master]
if {[llength $paths] == 0} { puts "ERROR: no master service"; exit 1 }
set m [lindex $paths 0]
open_service master $m
master_write_32 $m $REG_CAPCFG $N
master_write_32 $m $CAP_CTRL 0x1
set st [expr {[lindex [master_read_32 $m $CAP_CTRL 1] 0] & 0xffffffff}]
puts [format "ARMED: CAPCFG=%d CAP_STATUS=0x%08X" $N $st]
close_service master $m
