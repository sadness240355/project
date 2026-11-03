set m [file dirname [info script]]
set axis [get_clocks -quiet axis_aclk]
if {[llength $axis]!=1 || abs([get_property PERIOD $axis]-4.0)>0.00001} {error "Expected one 250MHz data clock"}
read_xdc [file join $m performance_margin_v7.xdc]
puts {PERFORMANCE_MARGIN_V7_APPLIED setup=0.100ns hold=0.010ns clock_period=4.000ns}
