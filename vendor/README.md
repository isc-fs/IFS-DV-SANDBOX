# Vendored packages (don't edit)

Copied from the team repos and built into the Docker image (`/opt/ifs_ws`).

| Package | Source | Version |
|---|---|---|
| `ifssim_bridge` | [IFSSIM](https://github.com/isc-fs/IFSSIM) `ros2/src/ifssim_bridge` | tag `v1.0.1` (matches the release binary) |
| `fs_msgs` | [IFS09-DV-PIPELINE](https://github.com/isc-fs/IFS09-DV-PIPELINE) `fs_msgs` | dev, 2026-10 |

Local change: both `package.xml` files get
`<export><build_type>ament_cmake</build_type></export>`. Without it colcon
builds them as plain CMake packages and ROS can't find them
(`package not found`).

To update: copy the new version over, keep that change, then
`docker compose build sandbox`.
