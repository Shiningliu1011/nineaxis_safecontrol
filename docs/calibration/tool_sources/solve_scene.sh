set -e
root="$(cd "$(dirname "$0")" && pwd)"
scene="$1"
source /opt/ros/humble/setup.bash
mkdir -p "$root/$scene-result"
timeout --signal=INT 12 "$root/fast-calib-build/fast_calib" \
  --ros-args -r __node:=fast_calib --params-file "$root/scene01.yaml" \
  -p bag_path:="$root/$scene" \
  -p image_path:="$root/$scene/rectified.png" \
  -p output_path:="$root/$scene-result" \
  -p z_max:=1.2 > "$root/$scene-solve.log" 2>&1
