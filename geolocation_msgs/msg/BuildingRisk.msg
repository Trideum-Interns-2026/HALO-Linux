# Building overflight risk for one drone position estimate. Bearing is from the drone to the
# building; distance is horizontal; height is the roof above the WGS84 ellipsoid.
std_msgs/Header header     # copied from the GeolocationState that was assessed

float64 probability            # dimensionless 0..1, chance the drone's 3-sigma position
                               # uncertainty overlaps the riskiest nearby building
float64 building_bearing_deg   # degrees clockwise from true north, drone to building, [0, 360)
float64 building_distance_m    # metres, horizontal, drone to the building's nearest edge; 0 if inside
float64 building_height_m      # metres, building roof height above the WGS84 ellipsoid
