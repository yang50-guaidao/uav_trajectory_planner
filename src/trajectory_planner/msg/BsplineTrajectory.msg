# B-spline trajectory message for trajectory execution
Header header

# Control points (flattened: x0,y0,z0,x1,y1,z1,...)
float64[] knots

# Time interval between knots
float64 dt

# Total trajectory duration
float64 duration

# Start execution time (ros::Time)
float64 start_time
