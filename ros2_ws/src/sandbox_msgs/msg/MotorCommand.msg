# Torque request for the two rear motors (one per wheel), normalised to the
# motor's maximum forward torque.
#   +1.0 = full forward torque
#    0.0 = coast
#   -0.2 = full regen (regen is limited to 20 % of the forward torque)
# Values outside [-0.2, 1.0] are clipped by the car.
std_msgs/Header header
float32 left
float32 right
