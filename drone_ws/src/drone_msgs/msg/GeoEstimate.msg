std_msgs/Header header        # stamp copied from the detection
float64 confidence
float64[3] lla               # lat (deg), lon (deg), alt (m, WGS84 ellipsoid)
float64[3] error_ecef        # 1-sigma error per ECEF axis (x, y, z) in meters
