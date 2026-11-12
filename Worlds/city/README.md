# City worlds

Two PX4-ready Gazebo worlds built on the city models from
[JdeRobot/RoboticsInfrastructure](https://github.com/JdeRobot/RoboticsInfrastructure) (branch
`humble-devel`). Each file is PX4's own default world (physics, gravity, ground plane, light, GPS
origin) plus one city model.

| World file | City model | Notes |
| --- | --- | --- |
| `city_basic.sdf` | `large_basic_city` | 500 m x 500 m, low buildings up to about 4 m, 2 MB |
| `city_cat_mouse.sdf` | `my_city_v3` | Blender export from the drone cat-and-mouse scene, 7 MB; the stock model draws it at 0.33 of its mesh size (165 m across) |
| `city_cat_mouse_full.sdf` | `my_city_v3`, inlined | the same city at the mesh's native size, 3x bigger (500 m across, trees about 30 m, mountains up to 78 m), ground plane 1500 m |

The world name equals the file name: PX4 loads `$PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf` and talks to
`/world/<name>/...`. The drone spawns at the world origin on the ground plane; the buildings sit on
top of it, so check that the origin is clear before flying (the first flight is by hand).

## One-time setup

The model meshes are not in this repo (about 2.4 GB for the whole checkout), so clone them into the
git-ignored `external/` folder:

```bash
mkdir -p /workspace/external && cd /workspace/external
git clone --depth 1 -b humble-devel https://github.com/JdeRobot/RoboticsInfrastructure
```

`.devcontainer/post-create.sh` then links the two worlds into PX4's world folder and adds the
models to `GZ_SIM_RESOURCE_PATH` (the models refer to their meshes as `model://large_basic_city/...`).
Re-run it after cloning, or do the same by hand:

```bash
M=/workspace/external/RoboticsInfrastructure/CustomRobots
export GZ_SIM_RESOURCE_PATH=$M/city_assets/models:$M/drone_cat_mouse/models:$GZ_SIM_RESOURCE_PATH
ln -sfn /workspace/Worlds/city/city_basic.sdf /opt/px4-gazebo/share/gz/worlds/city_basic.sdf
ln -sfn /workspace/Worlds/city/city_cat_mouse.sdf /opt/px4-gazebo/share/gz/worlds/city_cat_mouse.sdf
```

## Run

Pick the world with `PX4_GZ_WORLD`; everything else is the usual stack:

```bash
PX4_GZ_WORLD=city_basic fly                  # PX4 SITL + Gazebo in the basic city
PX4_GZ_WORLD=city_basic simstack up --gui    # the whole stack, left running
PX4_GZ_WORLD=city_cat_mouse fly              # the other city
PX4_GZ_WORLD=city_cat_mouse_full fly         # the other city at 3x the size
```

(`gz sim -r Worlds/city/city_basic.sdf` alone starts only Gazebo, without PX4 or the drone.)

## Scale

The drone is a real X500 (0.5 m across) and the person is 1.9 m tall, and Perception, Tracking and
Movement are tuned for that, so size the *city* up, not the drone down. A model include cannot
rescale a model, so `city_cat_mouse_full.sdf` inlines the city with its own `<scale>`: change the
three numbers in its `<mesh>` blocks (visual and collision must match) to any size, and keep the
ground plane larger than the mesh (the mesh is 500 m across at scale 1). The worlds are plain
files here: PX4 only loads the link in its world folder.
