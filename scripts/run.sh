#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CAMERA="${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${MONKEYS_LUNA:-/dev/ttyAMA2}"
NO_LUNA="${MONKEYS_NO_LUNA:-0}"
PI4_OV5647="${MONKEYS_PI4_OV5647:-0}"
FC="${MONKEYS_FC:-tcp://127.0.0.1:5760}"
CAMERA_YAML="${MONKEYS_CAMERA_YAML:-$ROOT/config/ov9281_current_mount.yaml}"
RUNTIME_JSON="${MONKEYS_RUNTIME_JSON:-$ROOT/config/runtime.json}"

CFG_FOCAL_SCALE="0.931"
CFG_FEATURE_ROI="0.20 0.32 0.80 0.90"
CFG_MAX_FEATURES="500"
CFG_LOCAL_GUI="1"
if [[ -f "$RUNTIME_JSON" ]]; then
  read -r CFG_FOCAL_SCALE RX0C RY0C RX1C RY1C CFG_MAX_FEATURES CFG_LOCAL_GUI < <(
    python3 - "$RUNTIME_JSON" <<'PY'
import json,sys
with open(sys.argv[1],"r",encoding="utf-8") as f:
    d=json.load(f)
roi=d.get("feature_roi",[0.20,0.32,0.80,0.90])
print(
    d.get("focal_scale",0.931),
    roi[0],roi[1],roi[2],roi[3],
    d.get("max_features",500),
    1 if d.get("local_gui",True) else 0
)
PY
  )
  CFG_FEATURE_ROI="$RX0C $RY0C $RX1C $RY1C"
fi

FOCAL_SCALE="${MONKEYS_FOCAL_SCALE:-$CFG_FOCAL_SCALE}"
FEATURE_ROI="${MONKEYS_FEATURE_ROI:-$CFG_FEATURE_ROI}"
MAX_FEATURES="${MONKEYS_MAX_FEATURES:-$CFG_MAX_FEATURES}"
LOCAL_GUI="${MONKEYS_LOCAL_GUI:-$CFG_LOCAL_GUI}"
GEOMETRY_JSON="${MONKEYS_GEOMETRY_JSON:-$ROOT/config/mount_geometry.json}"
[[ -f "$GEOMETRY_JSON" ]] || { echo "ОШИБКА: geometry config не найден: $GEOMETRY_JSON" >&2; exit 2; }
read -r CFG_CAMERA_X CFG_CAMERA_Y CFG_CAMERA_Z CFG_RANGE_Z < <(python3 - "$GEOMETRY_JSON" <<'PY'
import json,sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    g=json.load(f)
print(g["camera"]["x"], g["camera"]["y"], g["camera"]["z"], g["rangefinder"]["z"])
PY
)
CAMERA_X_M="${MONKEYS_CAMERA_X_M:-$CFG_CAMERA_X}"
CAMERA_Y_M="${MONKEYS_CAMERA_Y_M:-$CFG_CAMERA_Y}"
CAMERA_Z_M="${MONKEYS_CAMERA_Z_M:-$CFG_CAMERA_Z}"
RANGE_Z_M="${MONKEYS_RANGE_Z_M:-$CFG_RANGE_Z}"
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi
export MAVLINK_ROOT
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
CACHE_DIR="${MONKEYS_RUNTIME_CACHE_DIR:-$ROOT/build/runtime_cache}"
CACHED_BIN="$CACHE_DIR/monkeysstab_optical_flow"
mkdir -p "$RUN_ROOT" "$CACHE_DIR"

# Bound total storage used by automatically-created production runs before
# starting another logger. Named Web recordings/forensic datasets are excluded.
bash "$ROOT/scripts/log_retention.sh"

FREE_KB="$(df -Pk "$RUN_ROOT" | awk 'NR==2 {print $4}')"
FREE_MB=$((FREE_KB / 1024))
if (( FREE_MB < 300 )); then
  echo "ОШИБКА: недостаточно свободного места: ${FREE_MB} MB." >&2
  echo "Для запуска monkeysStab требуется минимум 300 MB." >&2
  exit 3
elif (( FREE_MB < 1024 )); then
  echo "ПРЕДУПРЕЖДЕНИЕ: на диске осталось только ${FREE_MB} MB (< 1 GB)." >&2
  echo "CSV будет автоматически остановлен при достижении лимита." >&2
fi

if [[ "${MONKEYS_FAST_RESTART:-0}" != "1" ]]; then
  if [[ "$PI4_OV5647" == "1" ]]; then
    echo "PI4: аудит геометрии RPi5 пропущен; параметры FC не изменяются."
    echo "ВНИМАНИЕ: геометрия установки OV5647 на Pi4 ещё не подтверждена."
  else
    bash "$ROOT/scripts/audit_geometry.sh"
  fi
  if [[ "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "1" || "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "true" || "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "yes" ]]; then
    if [[ "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "1" || "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "true" || "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "yes" ]]; then
      echo "ОШИБКА: MONKEYS_STABILISED_UNIFIED_PUBLISH и MONKEYS_RAW_UNIFIED_PUBLISH взаимоисключающие" >&2
      exit 2
    fi
    export MONKEYS_FLOW_OPTIONS_EXPECTED=1
  else
    export MONKEYS_FLOW_OPTIONS_EXPECTED=0
  fi
  if [[ "${MONKEYS_BUILD_ONLY:-0}" == "1" || "${MONKEYS_PI4_CAMERA_ONLY:-0}" == "1" ]]; then
    echo "BUILD ONLY: проверка параметров FC отложена; MAVLink и камера не запускаются."
  else
    bash "$ROOT/scripts/audit_fc_params.sh"
  fi
fi

if [[ "$PI4_OV5647" != "1" ]]; then
  [[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }
fi
if [[ "$NO_LUNA" != "1" ]]; then
  [[ -e "$LUNA" ]] || { echo "ОШИБКА: TF-Luna не найден: $LUNA" >&2; exit 2; }
fi
if [[ "$FC" != tcp://* ]]; then
  [[ -e "$FC" ]] || { echo "ОШИБКА: FC не найден: $FC" >&2; exit 2; }
fi
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: camera YAML не найден: $CAMERA_YAML" >&2; exit 2; }
[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || { echo "ОШИБКА: MAVLink headers не найдены: $MAVLINK_ROOT" >&2; exit 2; }
[[ -f "$ROOT/src/optical_flow_mavlink.cpp" ]] || {
  echo "ОШИБКА: отсутствует production source src/optical_flow_mavlink.cpp" >&2
  echo "Репозиторий перенесён не полностью. Не используйте старый jtzero-kimera как скрытую зависимость." >&2
  exit 2
}

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${MONKEYS_RUN_DIR:-$RUN_ROOT/${STAMP}_OPTICAL_FLOW}"
mkdir -p "$RUN_DIR"
BIN="$RUN_DIR/monkeysstab_optical_flow"
CSV="$RUN_DIR/optical_flow_mavlink.csv"
BUILD_LOG="$RUN_DIR/build.log"

# FORENSIC_DATASET_META_V1
# A dataset is an offline-replay artifact, not a second sensor reader.  Capture
# happens inside the production camera/Luna paths; here we freeze the exact
# software/configuration context needed to interpret it later.
if [[ -n "${MONKEYS_DATASET_DIR:-}" ]]; then
  mkdir -p "$MONKEYS_DATASET_DIR"
  DATASET_META="$MONKEYS_DATASET_DIR/session.json"
  DATASET_GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || true)"
  DATASET_GIT_DIRTY="$(git status --porcelain --untracked-files=no 2>/dev/null | wc -l | tr -d ' ')"
  export DATASET_META DATASET_GIT_COMMIT DATASET_GIT_DIRTY
  export DATASET_CAMERA="$CAMERA" DATASET_LUNA="$LUNA" DATASET_FC="$FC"
  export DATASET_CAMERA_YAML="$CAMERA_YAML" DATASET_GEOMETRY_JSON="$GEOMETRY_JSON"
  export DATASET_FOCAL_SCALE="$FOCAL_SCALE" DATASET_FEATURE_ROI="$FEATURE_ROI"
  export DATASET_MAX_FEATURES="$MAX_FEATURES" DATASET_RUN_DIR="$RUN_DIR"
  export DATASET_SURFACE="${MONKEYS_DATASET_SURFACE:-}"
  export DATASET_DURATION="${MONKEYS_DATASET_DURATION_SEC:-}"
  python3 - <<'PY'
import json, os, shutil, time
from pathlib import Path

dst=Path(os.environ["DATASET_META"]).parent
meta={
    "format":"monkeysStab-forensic-dataset-v1",
    "created_wall_ns":time.time_ns(),
    "git_commit":os.environ.get("DATASET_GIT_COMMIT",""),
    "git_dirty_tracked":int(os.environ.get("DATASET_GIT_DIRTY","0") or 0),
    "camera_device":os.environ["DATASET_CAMERA"],
    "luna_device":os.environ["DATASET_LUNA"],
    "fc_endpoint":os.environ["DATASET_FC"],
    "camera_yaml_source":os.environ["DATASET_CAMERA_YAML"],
    "geometry_json_source":os.environ["DATASET_GEOMETRY_JSON"],
    "focal_scale":float(os.environ["DATASET_FOCAL_SCALE"]),
    "feature_roi":[float(x) for x in os.environ["DATASET_FEATURE_ROI"].split()],
    "max_features":int(os.environ["DATASET_MAX_FEATURES"]),
    "surface":os.environ.get("DATASET_SURFACE",""),
    "duration_sec":float(os.environ["DATASET_DURATION"]) if os.environ.get("DATASET_DURATION") else None,
    "production_run_dir":os.environ["DATASET_RUN_DIR"],
    "files":{
        "frames_bin":"frames.mjpgbin",
        "frames_index":"frames.csv",
        "luna_raw":"luna_raw.csv",
        "camera_calibration":"camera_calibration.yaml",
        "mount_geometry":"mount_geometry.json",
    },
}
Path(os.environ["DATASET_META"]).write_text(
    json.dumps(meta,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
shutil.copy2(os.environ["DATASET_CAMERA_YAML"],dst/"camera_calibration.yaml")
shutil.copy2(os.environ["DATASET_GEOMETRY_JSON"],dst/"mount_geometry.json")
PY
fi

read -r RX0 RY0 RX1 RY1 <<< "$FEATURE_ROI"

if [[ "${MONKEYS_FAST_RESTART:-0}" == "1" ]]; then
  if [[ ! -x "$CACHED_BIN" ]]; then
    echo "ОШИБКА: быстрый restart запрошен, но проверенный runtime ещё не собран: $CACHED_BIN" >&2
    exit 4
  fi
  cp "$CACHED_BIN" "$BIN"
  echo "FAST RESTART: использую уже проверенный runtime $CACHED_BIN"
else
  if ! g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
    $(pkg-config --cflags opencv4 libcamera) -I"$MAVLINK_ROOT" -I"$ROOT/src" \
    "$ROOT/src/optical_flow_mavlink.cpp" -o "$BIN" \
    $(pkg-config --libs opencv4 libcamera) -lpthread >"$BUILD_LOG" 2>&1; then
    echo "ОШИБКА СБОРКИ. Последние 80 строк:"
    tail -80 "$BUILD_LOG"
    exit 1
  fi
  cp "$BIN" "$CACHED_BIN"
  chmod +x "$CACHED_BIN"
fi

if [[ "${MONKEYS_BUILD_ONLY:-0}" == "1" ]]; then
  echo "BUILD ONLY PASS: исходный runtime собран: $BIN"
  exit 0
fi

cat <<EOF
======================================================================
monkeysStab — OPTICAL FLOW
======================================================================
OV9281 -> OPTICAL_FLOW -> ArduPilot EKF3
TF-Luna -> DISTANCE_SENSOR

camera: $CAMERA
luna:   $LUNA
fc:     $FC
CSV:    $CSV

focal_scale: $FOCAL_SCALE
feature ROI: $FEATURE_ROI
max features: $MAX_FEATURES

Launcher НЕ ARM-ит FC и НЕ переключает режим полёта.
======================================================================
EOF

ARGS=(
  "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL_SCALE"
  --feature-roi "$RX0" "$RY0" "$RX1" "$RY1"
  --max-features "$MAX_FEATURES"
  --diag-camera-x-m "$CAMERA_X_M"
  --diag-camera-y-m "$CAMERA_Y_M"
  --diag-camera-z-m "$CAMERA_Z_M"
  --diag-range-z-m "$RANGE_Z_M"
)
if [[ "${MONKEYS_PI4_CAMERA_ONLY:-0}" == "1" ]]; then
  ARGS+=(--pi4-camera-only)
fi
if [[ "$PI4_OV5647" == "1" ]]; then
  ARGS+=(--pi4-ov5647)
fi
if [[ "$NO_LUNA" == "1" ]]; then
  ARGS+=(--no-luna)
  echo "NO-LUNA: аппаратный дальномер отсутствует. Без AGL метрическая навигация не подтверждена."
fi
if [[ "$LOCAL_GUI" == "1" || "$LOCAL_GUI" == "true" || "$LOCAL_GUI" == "yes" ]]; then
  ARGS+=(--rotation-gui)
fi
if [[ "${MONKEYS_RETURN_GUI:-0}" == "1" || "${MONKEYS_RETURN_GUI:-0}" == "true" || "${MONKEYS_RETURN_GUI:-0}" == "yes" ]]; then
  ARGS+=(--return-gui)
fi
if [[ "${MONKEYS_RETURN_CLI:-0}" == "1" || "${MONKEYS_RETURN_CLI:-0}" == "true" || "${MONKEYS_RETURN_CLI:-0}" == "yes" ]]; then
  ARGS+=(--return-cli)
fi
if [[ "${MONKEYS_BLIND4_CLI:-0}" == "1" || "${MONKEYS_BLIND4_CLI:-0}" == "true" || "${MONKEYS_BLIND4_CLI:-0}" == "yes" ]]; then
  ARGS+=(--blind4-cli)
fi
if [[ "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "1" || "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "true" || "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "yes" ]]; then
  ARGS+=(--return-manual-target)
fi
if [[ "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "1" || "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "true" || "${MONKEYS_STABILISED_UNIFIED_PUBLISH:-0}" == "yes" ]]; then
  ARGS+=(--stabilised-unified-publish)
fi
if [[ "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "1" || "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "true" || "${MONKEYS_RAW_UNIFIED_PUBLISH:-0}" == "yes" ]]; then
  ARGS+=(--raw-unified-publish)
fi
if [[ -n "${MONKEYS_DATASET_DIR:-}" ]]; then
  ARGS+=(--dataset-dir "$MONKEYS_DATASET_DIR")
fi
if [[ -n "${MONKEYS_DATASET_SURFACE:-}" ]]; then
  ARGS+=(--dataset-surface "$MONKEYS_DATASET_SURFACE")
fi
if [[ -n "${MONKEYS_DATASET_DURATION_SEC:-}" ]]; then
  ARGS+=(--dataset-duration-sec "$MONKEYS_DATASET_DURATION_SEC")
fi
if [[ -n "${MONKEYS_REMOTE_LOG:-}" ]]; then
  ARGS+=(--remote-log "$MONKEYS_REMOTE_LOG")
fi
if [[ -n "${MONKEYS_FB_SHADOW_MAX_PX:-}" ]]; then
  ARGS+=(--fb-shadow-max-px "$MONKEYS_FB_SHADOW_MAX_PX")
fi
if [[ "${MONKEYS_NO_OBS_SHADOW:-0}" == "1" ]]; then
  ARGS+=(--no-obs-shadow)
fi
if [[ -n "${MONKEYS_GUIDED_MM:-}" ]]; then
  ARGS+=(--guided-mm "$MONKEYS_GUIDED_MM")
fi
if [[ -n "${MONKEYS_CONTINUOUS_LEGS:-}" ]]; then
  ARGS+=(--continuous-legs "$MONKEYS_CONTINUOUS_LEGS")
fi
if [[ -n "${MONKEYS_PRE_STATIC_SEC:-}" ]]; then
  ARGS+=(--pre-static-sec "$MONKEYS_PRE_STATIC_SEC")
fi
if [[ -n "${MONKEYS_POST_STATIC_SEC:-}" ]]; then
  ARGS+=(--post-static-sec "$MONKEYS_POST_STATIC_SEC")
fi

exec "$BIN" "${ARGS[@]}"
