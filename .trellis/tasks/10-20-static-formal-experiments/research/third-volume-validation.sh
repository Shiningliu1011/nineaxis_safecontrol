set -e -o pipefail

mode=${1:?使用 --check-only 或 --execute}
repository=$(realpath "${2:?需要主仓库路径}")
source_run=$(realpath "${3:?需要已关闭的真实运行目录}")
engineering_evidence=$(realpath "${4:?需要工程容量证据路径}")
volume_root=/media/lsn/Windows/robot_safecontrol_static_experiments_20261021
control_output="$repository/output/static-capacity-20261020/third-volume-validation-01"
guard_output="$volume_root/storage-guard-01"
filesystem_output="$volume_root/filesystem-01"
script_path=$(realpath "${BASH_SOURCE[0]}")

test "$mode" = --check-only || test "$mode" = --execute
test -d "$volume_root"
test "$(realpath "$volume_root")" = "$volume_root"
test "$(findmnt --noheadings --raw --output UUID --target "$volume_root")" = 6C001EAA001E7B72
test "$(findmnt --noheadings --raw --output FSTYPE --target "$volume_root")" = ntfs3
mount_options=$(findmnt --noheadings --raw --output OPTIONS --target "$volume_root")
[[ ",$mount_options," == *,rw,* ]]
test -f "$source_run/plant.sqlite3"
test -f "$source_run/controller.sqlite3"
test -f "$source_run/source_configuration.yaml"
test -f "$source_run/manifest.json"
test -f "$source_run/result.json"
test -f "$engineering_evidence"
test ! -e "$control_output"
test ! -e "$guard_output"
test ! -e "$filesystem_output"
git -C "$repository" diff --exit-code 40d43bb -- src portable_oscbf
test -z "$(git -C "$repository" ls-files --others --exclude-standard -- src portable_oscbf)"
for entry in prepare_static_storage_budget.py validate_static_storage_guard.py validate_static_filesystem.py; do
    test -f "$repository/scripts/$entry"
done
findmnt --json --target "$volume_root" --output SOURCE,TARGET,FSTYPE,OPTIONS,UUID
df --block-size=1 "$volume_root" "$repository"
if test "$mode" = --check-only; then
    printf '准备检查通过；尚未执行记录写入、SIGKILL 或 sim。\n'
    exit 0
fi

source "$repository/.scratch/host-recovery/env.sh"
set -u
export PYTHONPATH="$repository/src:$repository/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}"
cd "$repository"
mkdir "$control_output"
exec > >(tee "$control_output/execution.log") 2>&1
trap 'result=$?; date --iso-8601=seconds > "$control_output/finished.txt"; printf "%s\n" "$result" > "$control_output/exit-code.txt"' EXIT
date --iso-8601=seconds > "$control_output/started.txt"
printf '%q ' bash "$script_path" "$mode" "$repository" "$source_run" "$engineering_evidence" > "$control_output/command.txt"
printf '\n' >> "$control_output/command.txt"
taskset --pid --cpu-list $$ > "$control_output/cpu-affinity.txt"
git rev-parse HEAD > "$control_output/git-commit.txt"
cp "$script_path" "$control_output/execution-source.sh"
findmnt --json --target "$volume_root" --output SOURCE,TARGET,FSTYPE,OPTIONS,UUID > "$control_output/filesystem.json"
sha256sum "$script_path" "$engineering_evidence" "$source_run/manifest.json" \
    "$source_run/source_configuration.yaml" \
    scripts/prepare_static_storage_budget.py scripts/validate_static_storage_guard.py \
    scripts/validate_static_filesystem.py scripts/validate_static_recording_upgrade.py \
    > "$control_output/input-sha256.txt"
df --block-size=1 "$volume_root" "$repository" > "$control_output/space-before.txt"

python scripts/prepare_static_storage_budget.py \
    --engineering-evidence "$engineering_evidence" --resource-root "$repository" \
    --config "$source_run/source_configuration.yaml" --method E --feed-factor 1 \
    --maximum-pairs 66 --volume "$volume_root" \
    --output "$control_output/e-s5.storage.json"
python scripts/validate_static_storage_guard.py \
    --source "$source_run" --storage-budget "$control_output/e-s5.storage.json" \
    --output "$guard_output"
python scripts/validate_static_filesystem.py --run "$source_run" --output "$filesystem_output"
sha256sum "$control_output/e-s5.storage.json" "$guard_output/validation.json" \
    "$filesystem_output/validation.json" "$filesystem_output/atomic.json" \
    > "$control_output/result-sha256.txt"
df --block-size=1 "$volume_root" "$repository" > "$control_output/space-after.txt"
printf '第三卷离线验证命令全部完成；完整 sim 与三卷容量生成仍需执行。\n'
