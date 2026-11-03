# Drone GPS visualization

Run `gps` in a terminal to read the connected PX4 vehicle's GPS position from
MAVSDK and publish ROS visualization markers on `/drone_gps/markers`. The
markers use a local ENU `map` frame anchored at the first valid GPS fix and use
MAVSDK's absolute altitude (AMSL). Start RViz separately, set its fixed frame
to `map`, and add a `MarkerArray` display for `/drone_gps/markers`.

MAVSDK's position message does not provide meter-valued GPS accuracy. By
default, the node therefore publishes the raw GPS marker only and reports that
filtered uncertainty is unavailable. It does not interpret HDOP/VDOP as meters
or invent a GPS accuracy value.

To enable the position Kalman filter and uncertainty display, provide the GPS
receiver's documented one-sigma standard deviations in meters:

```bash
gps --ros-args \
  -p horizontal_stddev_m:=1.0 \
  -p vertical_stddev_m:=2.0
```

The values above only illustrate the command syntax; replace them with the
receiver's documented values. They are not recommended or assumed GPS
accuracies.

With both values configured, the node filters position, draws a translucent
3-sigma sphere whose radius uses the larger horizontal standard deviation,
and shows vertical `+/- 3 sigma` separately. The process-noise variance is
configurable with `process_noise_variance_m2_per_s` (default `0.25`). The
node requests position telemetry at 5 Hz and marks the retained position
stale after 1 second without a valid fix; the filter is not extrapolated.
This output is for visualization only and does not affect vehicle control or
navigation. DTED support and downloads are not part of this feature.
