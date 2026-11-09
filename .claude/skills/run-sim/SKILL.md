---
name: run-sim
description: Only for sim-operator, and only when a unit test cannot prove the claim. Runs the scripted end-to-end sim check (PX4 SITL + Gazebo + all four services, headless or with the GUI) and reports its table.
---
The whole procedure is one script, `/workspace/scripts/sim_check`. It starts the sim and every
service, takes the readings, arms and flies the vehicle, runs Movement, lands, and tears everything
down, even after a failure. Do not redo its steps by hand and never edit a file or change PX4
parameters. Arming and takeoff are allowed ONLY in this sim and only through this script or
`/workspace/scripts/sitl_vehicle`.

Every Bash command starts with `export HALO_SIM_OPERATOR=1;`.

1. Make sure nothing is running: `pgrep -af 'px4|gz[ ]sim|MicroXRCE'` must print nothing, else tell
   the caller (sim_check refuses to start next to another sim).
2. Run it, logging to a file so a long run does not flood the output (it takes 2-4 minutes, so use
   a generous timeout, e.g. 600 s):
   - headless: `/workspace/scripts/sim_check > /tmp/halo-sim_check.out 2>&1`
   - GUI on the Selkies desktop: `/workspace/scripts/sim_check --gui ...`
   - GUI with pauses so a human can watch: `/workspace/scripts/sim_check --watch ...`
   - Movement orbits the person (30 s per lap) instead of following: add `--orbit`.
   - no flight: add `--skip-flight`.
   For the GUI the display must be up: `DISPLAY=:0 xdpyinfo | head -3` must work.
3. Report the final table from the end of /tmp/halo-sim_check.out verbatim (PASS/FAIL/WARN/SKIP per
   check, with details), plus the exit code. For every FAIL quote the matching lines of
   /tmp/halo-check/<name>.log (fly, sim_links, geolocation, tracking, perception, movement, link),
   stripping the `pxh>` noise with `grep -av`.
4. Confirm teardown: the table's `teardown` row, plus
   `pgrep -af 'px4|ruby|gz[ ]sim|MicroXRCE|ros2|parameter_bridge'` printing nothing.
