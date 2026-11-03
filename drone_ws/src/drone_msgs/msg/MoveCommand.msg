std_msgs/Header header
uint8 GOTO=0
uint8 ORBIT=1
uint8 type
float64 lat
float64 lon
float64 alt                   # commanded drone altitude (same frame as /drone/gps)
float64 radius                # ORBIT only, meters
