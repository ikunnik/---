# Version 1. source.stamp is the original capture stamp, never receive time.
uint8 CLOCK_UNKNOWN=0
uint8 CLOCK_SYSTEM=1
uint8 CLOCK_ROS=2
uint8 CLOCK_DEVICE=3
std_msgs/Header source
bool source_stamp_valid
uint8 clock_domain
# UUID of acquisition session; reset sequence only with a new stream_id.
string stream_id
uint64 sequence
# False means detector/input failure; true + visible=false means no target.
bool detector_ok
