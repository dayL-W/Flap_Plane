uint32 timestamp_ms		# in milliseconds since system start
char[20] key			# max. 10 characters as key / name
float32 value			# the value to send as debug output

float32 rollspeed	# Bias corrected angular velocity about X body axis in rad/s
float32 pitchspeed	# Bias corrected angular velocity about Y body axis in rad/s
float32 yawspeed	# Bias corrected angular velocity about Z body axis in rad/s

float32[4] q		# Quaternion rotation from NED earth frame to XYZ body frame
float32[4] delta_q_reset 	# Amount by which quaternion has changed during last reset
uint8 quat_reset_counter	# Quaternion reset counter
