# aqua_msgs/msg/PipeSensor.msg

uint32 sensor_id
uint32 pipe_id
uint8  pipe_type        # 0=fresh, 1=sewage

float32 pressure        # bar
float32 level           # percent (0–100)

bool valve_state

builtin_interfaces/Time stamp
