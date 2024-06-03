# reference for polynomial trajectory tracking
#

std_msgs/Header header

uint8 type_mask
uint8 flag 	# Position Velocity Acceleration Jerk Reference


geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 refpose
geometry_msgs/Vector3 refvel
geometry_msgs/Vector3 output
geometry_msgs/Vector3 output_filter
geometry_msgs/Vector3 thrust
geometry_msgs/Vector3 thrust_filter
