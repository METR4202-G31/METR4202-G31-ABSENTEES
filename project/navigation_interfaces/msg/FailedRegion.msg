uint8 PLANNING_FAILED=0
uint8 NAVIGATION_FAILED=1
uint8 STUCK=2
uint8 NAVIGATION_TIMEOUT=3

uint32 region_id

geometry_msgs/Point center
float32 radius

uint32 failure_count
uint8 last_failure_reason

builtin_interfaces/Time first_failure_time
builtin_interfaces/Time last_failure_time