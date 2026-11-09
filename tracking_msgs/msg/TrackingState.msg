# Person position estimate. Position is WGS84 LLA; errors are per-axis 3-sigma
# bounds in Earth-Centred, Earth-Fixed (ECEF) metres.
std_msgs/Header header

float64 latitude_deg       # WGS84 latitude, degrees
float64 longitude_deg      # WGS84 longitude, degrees
float64 altitude_m         # WGS84 ellipsoid altitude, metres
float64 error_x_m          # ECEF X-axis 3-sigma bound, metres
float64 error_y_m          # ECEF Y-axis 3-sigma bound, metres
float64 error_z_m          # ECEF Z-axis 3-sigma bound, metres
