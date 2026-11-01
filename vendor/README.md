# Vendored packages (don't edit)

Copied from the team repos and built into the Docker image (`/opt/ifs_ws`).

| Package | Source | Version |
|---|---|---|
| `ifssim_bridge` | [IFSSIM](https://github.com/isc-fs/IFSSIM) `ros2/src/ifssim_bridge` | tag `v1.0.1` (matches the release binary) |
| `fs_msgs` | [IFS09-DV-PIPELINE](https://github.com/isc-fs/IFS09-DV-PIPELINE) `fs_msgs` | dev, 2026-10 |
| `ifs09_cone_detection` | [IFS09-DV-PIPELINE](https://github.com/isc-fs/IFS09-DV-PIPELINE) `cone_detection/cone_detection` + `LICENSE` | dev `f58d9e5` (same as the reference scores) |

Local change: the `ifssim_bridge` and `fs_msgs` `package.xml` files get
`<export><build_type>ament_cmake</build_type></export>`. Without it colcon
builds them as plain CMake packages and ROS can't find them
(`package not found`).

To update: copy the new version over, keep that change, then
`docker compose build sandbox`.

## `ifs09_cone_detection`: the real perception

The real pipeline's cone-detection algorithm, so students can run it
(`perception:=ifs09`) and start their own perception from it
(`sb-new <name> --from ifs09`, which copies this folder into their package).

- The Python package keeps its real name, `cone_detection`, and its files
  are unchanged.
- Left out: `cone_detection_node.py` (the lifecycle node: needs `node_base`,
  `dv_msgs`, `mode_manager`). `sandbox_pipeline/perception_ifs09.py` replaces
  it on the sandbox topics, with `sandbox_tools/ifs09.py` as glue.
- Added: `package.xml` + `setup.py` (ROS package `ifs09_cone_detection`).
- Its Python libraries are pinned in `docker/Dockerfile` and
  `.github/workflows/ci.yml`, at the versions of the pipeline's image
  (IFSSIM repo, `docker/dv_pipeline_stack`).

To update (do it together with the reference scores, so `perception:=ifs09`
and the "real pipeline" rows are the same code):

```bash
# on your computer, from the repo root
COMMIT=<pipeline commit>
rm -rf vendor/ifs09_cone_detection/cone_detection
git -C ../IFS09-DV-PIPELINE archive "$COMMIT" cone_detection/cone_detection cone_detection/LICENSE \
    | tar -x -C vendor/ifs09_cone_detection --strip-components=1
rm vendor/ifs09_cone_detection/cone_detection/cone_detection_node.py
docker compose up -d --build sandbox
# then in the sandbox shell
sb-build && sb-test && sb-score perception:=ifs09 --scenario mock-all
```

Then update the commit in the table above. Student copies made before stay
as they are (they're the students' code from then on).
