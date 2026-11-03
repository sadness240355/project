# Implementation design margin, without changing any clock period.
# These additional budgets apply only to synchronous 250MHz data-domain paths.
set_clock_uncertainty -setup 0.100 -from [get_clocks axis_aclk] -to [get_clocks axis_aclk]
set_clock_uncertainty -hold 0.010 -from [get_clocks axis_aclk] -to [get_clocks axis_aclk]
