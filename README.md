# M-detector Offline Prototype

This repository packages the M-detector moving object detector as a standalone
offline CMake project. It processes sequential lidar PCD folders, estimates a
local lidar pose with `direct_lidar_odometry`, segments dynamic and static
points, and writes per-frame plus merged PCD outputs for visualization.

The original M-detector paper is:

> Moving Event Detection from LiDAR Stream Points, Nature Communications, 2024.

This prototype intentionally does not require ROS, catkin, Livox drivers,
Fast-LIO, or a manually installed oneTBB tree.

## Repository Layout

```text
.
  CMakeLists.txt
  .devcontainer/
  config/offline/aiv_at128.yaml
  direct_lidar_odometry/        # Git submodule: upstream lidar odometry
  include/
  src/
  tests/
  data/                         # local datasets, ignored by git
  output/                       # generated results, ignored by git
```

`direct_lidar_odometry` is tracked as a git submodule at
`https://github.com/Nicky0325/direct_lidar_odometry.git`. Keep it clean against
its upstream unless you are deliberately updating the submodule pointer.

## Clone And Update

Fresh clone:

```bash
git clone --recurse-submodules <this-repository-url>
cd M-detector
```

Existing clone:

```bash
git submodule update --init --recursive
```

When pulling later:

```bash
git pull --recurse-submodules
git submodule update --init --recursive
```

Optional convenience:

```bash
git config submodule.recurse true
```

To verify that the odometry module is still an upstream-clean checkout:

```bash
git -C direct_lidar_odometry status --short --branch
git -C direct_lidar_odometry diff --stat origin/master
```

## Dependencies

Use the devcontainer, or any Ubuntu 22.04 environment with:

- CMake 3.16+
- Ninja
- C++17 compiler
- Eigen3
- PCL
- Boost filesystem/system
- yaml-cpp
- OpenMP
- TBB from the distro package, for example `libtbb-dev`

No ROS environment is required.

## Devcontainer

The devcontainer uses the multi-arch `ubuntu:22.04` image, so it builds natively
on Apple Silicon Docker Desktop. It installs only standalone C++ dependencies
and uses distro `libtbb-dev`.

With VS Code or another Dev Containers compatible tool, open the repository and
choose **Reopen in Container**. After the container starts:

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Docker CLI equivalent:

```bash
docker build -t m-detector-offline -f .devcontainer/Dockerfile .
docker run --rm -it \
  -v "$PWD":/workspaces/M-detector \
  -w /workspaces/M-detector \
  m-detector-offline bash
```

Then run the same `git submodule`, `cmake`, `cmake --build`, and `ctest`
commands inside the container.

## Build

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The build produces:

```text
build/m_detector_offline
```

## Dataset Format

The offline runner expects one sequence directory with lidar folders under
`lidar/`:

```text
data/<sequence_name>/
  lidar/
    lidar_frf_at128/
      1776745760.800996.pcd
      1776745760.900821.pcd
      ...
    lidar_blb_at128/
      1776745760.800996.pcd
      1776745760.900821.pcd
      ...
  calibration/                  # optional for this prototype
  camera/                       # optional for this prototype
  vehicle_pose/                 # optional for this prototype
```

Only the selected lidar PCD folders are consumed. PCD files must be readable by
PCL as `pcl::PointXYZI`; `x`, `y`, `z`, and `intensity` fields are recommended.
The runner sorts frames numerically by the timestamp in each PCD filename stem.

The sample workspace used during development contains:

```text
data/5073_sequential/lidar/lidar_frf_at128
data/5073_sequential/lidar/lidar_blb_at128
```

## Run

Process the recommended AT128 lidars independently:

```bash
./build/m_detector_offline \
  --data data/5073_sequential \
  --lidars lidar_frf_at128,lidar_blb_at128 \
  --config config/offline/aiv_at128.yaml \
  --output output/5073_sequential
```

Quick smoke run:

```bash
./build/m_detector_offline \
  --data data/5073_sequential \
  --lidars lidar_frf_at128 \
  --config config/offline/aiv_at128.yaml \
  --output /tmp/mdetector_smoke \
  --max-frames 5
```

CLI options:

```text
--data <sequence_dir>       Sequence root containing lidar/<lidar_name> folders.
--lidars <a,b,c>            Comma-separated lidar folder names.
--config <yaml>             Detector config, default config/offline/aiv_at128.yaml.
--output <dir>              Output root.
--max-frames <N>            Optional frame limit for smoke tests.
--help                      Print usage.
```

## Output

Each lidar gets its own output tree:

```text
output/<sequence>/<lidar_name>/
  odometry/
    local_pose.txt
    local_map_keyframes.pcd
  dynamic_frames/
    <timestamp>.pcd
  static_frames/
    <timestamp>.pcd
  dynamic_object_transformed/
    <timestamp>.pcd
    merged.pcd
  static_object_transformed/
    <timestamp>.pcd
    merged.pcd
  dynamic_merged.pcd
  summary.csv
```

`dynamic_frames` and `static_frames` contain the detector's per-frame outputs.
The `*_object_transformed` folders contain final segmented points transformed by
the lidar odometry pose; each folder also includes a concatenated `merged.pcd`
for easier visualization in tools such as CloudCompare or PCL viewers.

`summary.csv` contains one row per processed frame:

```text
frame_id,file_name,timestamp,input_points,dynamic_points,static_points,tx,ty,tz,qx,qy,qz,qw,total_time_sec
```

`odometry/local_pose.txt` stores the local pose used for each frame.

## Development Notes

- `lidar_frf_at128` and `lidar_blb_at128` are processed as independent
  sequences, not fused.
- Detector parameters live in `config/offline/aiv_at128.yaml`.
- The M-detector core is initialized from YAML rather than ROS parameters.
- Prefer parent-project wrappers or adapters over local edits inside
  `direct_lidar_odometry`.
- Keep build artifacts, datasets, generated outputs, and local paper PDFs out of
  git.

## License

The original source code is released under the GPLv2 license for academic usage.
For commercial use, contact the original authors.
