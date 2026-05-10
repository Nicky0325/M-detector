# AGENTS.md

Guidance for future coding agents working in this repository.

## Project Goal

This is an offline, standalone prototype of M-detector. Preserve the current
direction:

- No ROS, catkin, Livox ROS driver, Fast-LIO, or manually installed oneTBB.
- Build with CMake/Ninja and distro dependencies on Ubuntu 22.04.
- Run on sequential lidar PCD folders from `data/<sequence>/lidar/<lidar_name>`.
- Use `direct_lidar_odometry` for local lidar pose and M-detector for dynamic
  and static segmentation.

## Submodule Boundary

`direct_lidar_odometry/` is a git submodule that points to:

```text
https://github.com/Nicky0325/direct_lidar_odometry.git
```

Treat it as read-only upstream code by default. Before finishing any task that
touches odometry, verify:

```bash
git -C direct_lidar_odometry status --short --branch
git -C direct_lidar_odometry diff --stat origin/master
```

Expected result: clean worktree and no diff against the fetched upstream commit.
If odometry changes are truly required, prefer one of these paths:

- Add an adapter or wrapper in this parent repository.
- Open a change upstream and update the submodule pointer after it lands.
- Use a deliberate fork and update `.gitmodules` only with explicit user
  approval.

Do not leave local patches inside the submodule for publication.

## Build And Test

Primary verification commands:

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Smoke test when data is available:

```bash
./build/m_detector_offline \
  --data data/5073_sequential \
  --lidars lidar_frf_at128 \
  --config config/offline/aiv_at128.yaml \
  --output /tmp/mdetector_smoke \
  --max-frames 5
```

On Apple Silicon, use the devcontainer or Docker image from
`.devcontainer/Dockerfile`; do not force x86 emulation unless the user asks.

## Dataset Contract

The runner consumes PCD files from:

```text
data/<sequence>/lidar/<lidar_name>/<timestamp>.pcd
```

Filenames are sorted numerically by timestamp. PCD files should load as
`pcl::PointXYZI`. Calibration, camera, and vehicle-pose folders may exist in the
dataset but are not consumed by the current offline prototype.

## Output Contract

For each processed lidar, keep these outputs stable:

```text
<output>/<lidar_name>/
  odometry/local_pose.txt
  odometry/local_map_keyframes.pcd
  dynamic_frames/<timestamp>.pcd
  static_frames/<timestamp>.pcd
  dynamic_object_transformed/<timestamp>.pcd
  dynamic_object_transformed/merged.pcd
  static_object_transformed/<timestamp>.pcd
  static_object_transformed/merged.pcd
  dynamic_merged.pcd
  summary.csv
```

The transformed folders should contain local-frame segmented points transformed
by the corresponding lidar odometry pose, plus concatenated `merged.pcd` files
for visualization.

## Coding Notes

- Follow existing C++17 style and keep edits narrowly scoped.
- Use PCL/Eigen/yaml-cpp/Boost APIs rather than ad hoc parsers for structured
  data.
- Keep generated outputs, datasets, build folders, and local PDFs out of git.
- Add or update tests under `tests/` when changing parsing, config loading,
  output paths, or point-cloud transformation behavior.
- If changing CLI behavior, update `README.md` in the same change.
