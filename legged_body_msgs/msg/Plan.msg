# This message holds a motion plan (i.e. trajectory of states and controls)
# Accurate timing information is stored in the header

Header header
time plan_timestamp
float64 compute_time
float64[] times
State[] states
Control[] controls

