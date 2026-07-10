project_open bw -revision bw
create_timing_netlist -model slow
read_sdc
update_timing_netlist
set res [report_timing -setup -npaths 9 -detail summary -stdout]
project_close
