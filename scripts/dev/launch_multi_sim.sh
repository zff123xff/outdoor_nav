#!/usr/bin/env bash
# Launch several navigation-simulation instances (one namespace each), one
# terminal per instance, against an already running UE/BeamNG simulator.
#
# Design notes:
#   * Every instance runs `robot_launch simulation.launch.py` with its own
#     robot_namespace; the simulator itself is external (sim_ws/simulation/start.sh).
#   * The UE<->ROS UDP bridges, YOLO and RTMP nodes bind fixed ports and the
#     shared GPU, so by default only the first instance starts the bridges and
#     none of them start YOLO/video.
#   * Navigation on the board runs as root: a nhy-owned node cannot open the
#     root-owned FastDDS shared-memory segment, so tasks published by root-side
#     bridges would never arrive. This script therefore re-executes itself
#     through sudo unless --as-user is given.
#   * Terminal muxer is auto-detected (gnome-terminal -> xterm -> log mode).
#     Log mode works over SSH with no X session at all.
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SETUP="$REPO/install/local_setup.bash"
# Fixed path so the sudo re-exec and a later `--stop` share the same bookkeeping.
STATE_DIR="${OUTDOOR_NAV_MULTI_STATE:-/tmp/outdoor_nav_multi}"
LOG_DIR="$STATE_DIR/logs"
PID_DIR="$STATE_DIR/pids"
NS_FILE="$STATE_DIR/namespaces"
TITLE_PREFIX="navsim"

COUNT=2
NAMESPACES=""
PREFIX="robot_"
START_INDEX=1
MODE="auto"
MODE_RESOLVED=""
RVIZ_FIRST="true"
RVIZ_ALL="false"
BRIDGES_ALL="false"
YOLO="false"
VIDEO="false"
SIM_TIME="false"
ROS_DOMAIN=""
ACTION="start"
DRY_RUN="false"
FOLLOW="false"
AS_USER="false"
EXTRA_ARGS=()
ORIG_ARGS=("$@")

usage() {
    cat <<'EOF'
用法: bash scripts/dev/launch_multi_sim.sh [选项] [-- 额外 launch 参数]

  -n N              实例数量（默认 2）
  --namespaces a,b  显式指定命名空间（覆盖 -n/--prefix/--start）
  --prefix NAME     命名空间前缀（默认 robot_）
  --start N         起始编号（默认 1，得到 robot_1、robot_2 …）
  --mode MODE       auto | gnome | xterm | log（默认 auto）
  --rviz-first      仅第一个实例开 RViz（默认）
  --rviz-all        每个实例都开 RViz
  --bridges-all     每个实例都启动 MavrosBridge（默认只有第一个）
  --yolo            所有实例启动 YOLO（默认关，TensorRT 会抢 GPU）
  --video           所有实例启动 RTMP 图传（默认关，端口会冲突）
  --sim-time        传 use_sim_time:=true
  --domain ID       设置 ROS_DOMAIN_ID（默认沿用当前环境）
  --follow          启动后在本终端 tail -f 所有日志
  --check|--dry-run 只做预检并打印将要执行的命令，不启动
  --stop            停止本脚本启动过的实例（读取上次的命名空间清单）
  --as-user         不通过 sudo 提权（以当前用户运行）
  -h, --help        显示本帮助

示例:
  bash scripts/dev/launch_multi_sim.sh                      # robot_1 + robot_2
  bash scripts/dev/launch_multi_sim.sh -n 4                 # robot_1..robot_4
  bash scripts/dev/launch_multi_sim.sh --namespaces robot_1,robot_7 --rviz-all
  bash scripts/dev/launch_multi_sim.sh -n 3 -- yolo:=false  # 额外参数原样透传
  bash scripts/dev/launch_multi_sim.sh --stop               # 全部停止

说明:
  * 仿真器不在这里启动：先在 sim_ws 里执行 simulation/start.sh，并保持同一
    ROS_DOMAIN_ID、同一 GPS 原点。
  * 默认第一个实例带 MavrosBridge（UDP 端口唯一），其余实例
    use_mavros_bridge:=false；YOLO/图传默认全部关闭。
  * log 模式适合 SSH：日志在 ~/.local/state/outdoor_nav_multi/logs/<ns>.log。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n) COUNT="${2:?}"; shift 2 ;;
        --namespaces) NAMESPACES="${2:?}"; shift 2 ;;
        --prefix) PREFIX="${2:?}"; shift 2 ;;
        --start) START_INDEX="${2:?}"; shift 2 ;;
        --mode) MODE="${2:?}"; shift 2 ;;
        --rviz-first) RVIZ_FIRST=true; RVIZ_ALL=false; shift ;;
        --rviz-all) RVIZ_FIRST=true; RVIZ_ALL=true; shift ;;
        --bridges-all) BRIDGES_ALL=true; shift ;;
        --yolo) YOLO=true; shift ;;
        --video) VIDEO=true; shift ;;
        --sim-time) SIM_TIME=true; shift ;;
        --domain) ROS_DOMAIN="${2:?}"; shift 2 ;;
        --follow) FOLLOW=true; shift ;;
        --check|--dry-run) DRY_RUN=true; shift ;;
        --stop) ACTION="stop"; shift ;;
        --as-user) AS_USER=true; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; EXTRA_ARGS=("$@"); break ;;
        *) echo "未知参数: $1（用 --help 查看用法；额外 launch 参数请放在 -- 之后）" >&2; exit 2 ;;
    esac
done

say()  { printf '%s\n' "$*"; }
warn() { printf '[warn] %s\n' "$*" >&2; }
die()  { printf '[error] %s\n' "$*" >&2; exit 1; }

[[ "$COUNT" =~ ^[1-9][0-9]*$ ]] || die "-n 必须是正整数"
[[ "$START_INDEX" =~ ^[0-9]+$ ]] || die "--start 必须是非负整数"
[[ -r "$SETUP" ]] || die "找不到 $SETUP，请先在该工作区执行 build.sh"

if [[ -n "$ROS_DOMAIN" ]]; then
    [[ "$ROS_DOMAIN" =~ ^[0-9]+$ ]] || die "--domain 必须是整数"
    export ROS_DOMAIN_ID="$ROS_DOMAIN"
fi

# ---------------------------------------------------------------- namespaces
NS_LIST=()

resolve_namespaces() {
    local -a list=()
    if [[ -n "$NAMESPACES" ]]; then
        IFS=',' read -r -a list <<< "$NAMESPACES"
    else
        local i
        for ((i = 0; i < COUNT; i++)); do
            list+=("${PREFIX}$((START_INDEX + i))")
        done
    fi
    local ns seen=""
    NS_LIST=()
    for ns in "${list[@]}"; do
        ns="${ns// /}"
        [[ -z "$ns" ]] && continue
        [[ "$ns" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] ||
            die "命名空间 '$ns' 非法：只能含字母数字下划线且以字母开头"
        if [[ ",$seen," == *",$ns,"* ]]; then
            die "命名空间重复: $ns"
        fi
        seen="$seen,$ns"
        NS_LIST+=("$ns")
    done
    ((${#NS_LIST[@]} > 0)) || die "没有解析到任何命名空间"
}

# ---------------------------------------------------------------- display / mode
detect_display() {
    if [[ -n "${DISPLAY:-}" ]]; then
        return 0
    fi
    local uid display candidate
    uid="$(id -u)"
    display="$(loginctl show-user "$(id -un)" -p Display --value 2>/dev/null | head -1)"
    if [[ -n "$display" ]]; then
        export DISPLAY="$display"
        for candidate in "/run/user/$uid/gdm/Xauthority" "$HOME/.Xauthority"; do
            if [[ -r "$candidate" ]]; then
                export XAUTHORITY="$candidate"
                break
            fi
        done
        return 0
    fi
    return 1
}

resolve_mode() {
    case "$MODE" in
        gnome|xterm|log) MODE_RESOLVED="$MODE"; return 0 ;;
        auto) ;;
        *) die "--mode 只能是 auto/gnome/xterm/log" ;;
    esac
    if command -v gnome-terminal >/dev/null 2>&1 && detect_display; then
        MODE_RESOLVED=gnome
    elif command -v xterm >/dev/null 2>&1 && detect_display; then
        MODE_RESOLVED=xterm
    else
        MODE_RESOLVED=log
    fi
}

# ---------------------------------------------------------------- privileges
maybe_sudo() {
    [[ "$AS_USER" == true || "$DRY_RUN" == true ]] && return 0
    [[ "$EUID" -eq 0 ]] && return 0
    if [[ -t 0 ]]; then
        say "[sudo] 板端导航以 root 运行，非 root 节点收不到 root 侧任务；改用 sudo 重新执行"
        exec sudo -E bash "${BASH_SOURCE[0]}" "${ORIG_ARGS[@]}"
    fi
    warn "当前非 root 且无交互终端，继续以 $(id -un) 运行；跨用户的 FastDDS 共享内存可能收不到数据"
}

# ---------------------------------------------------------------- preflight
preflight() {
    local ns
    for ns in "${NS_LIST[@]}"; do
        if pgrep -f "simulation.launch.py.*robot_namespace:=$ns" >/dev/null 2>&1; then
            warn "命名空间 $ns 已有 simulation.launch.py 在运行，重复启动会出现节点重名"
        fi
    done
    if [[ "$BRIDGES_ALL" == true ]] && pgrep -f "mavros_bridge_supervisor" >/dev/null 2>&1; then
        warn "已有 mavros_bridge_supervisor 在运行；--bridges-all 会抢占同一 UDP 端口"
    fi
}

# ---------------------------------------------------------------- commands
launch_cmd() {
    local ns="$1" index="$2" rviz="$3" a quoted=""
    local -a args=(
        "robot_namespace:=$ns"
        "use_sim_time:=$SIM_TIME"
        "rviz:=$rviz"
        "yolo:=$YOLO"
        "video_stream:=$VIDEO"
    )
    if [[ "$index" -ne 0 && "$BRIDGES_ALL" != true ]]; then
        args+=("use_mavros_bridge:=false")
    fi
    if ((${#EXTRA_ARGS[@]})); then
        args+=("${EXTRA_ARGS[@]}")
    fi
    for a in "${args[@]}"; do
        quoted+=" $(printf '%q' "$a")"
    done
    printf 'source %q && cd %q && exec ros2 launch robot_launch simulation.launch.py%s' \
        "$SETUP" "$REPO" "$quoted"
}

instance_rviz() {
    local index="$1"
    if [[ "$RVIZ_ALL" == true ]] || { [[ "$RVIZ_FIRST" == true ]] && [[ "$index" -eq 0 ]]; }; then
        printf 'true'
    else
        printf 'false'
    fi
}

instance_bridges() {
    local index="$1"
    if [[ "$index" -eq 0 || "$BRIDGES_ALL" == true ]]; then
        printf 'on'
    else
        printf 'off'
    fi
}

launch_instance() {
    local ns="$1" index="$2" rviz cmd log env
    rviz="$(instance_rviz "$index")"
    cmd="$(launch_cmd "$ns" "$index" "$rviz")"
    log="$LOG_DIR/$ns.log"
    env="export ROS_LOG_DIR='$LOG_DIR/$ns' ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-0}'"
    : > "$log"

    case "$MODE_RESOLVED" in
        gnome)
            gnome-terminal --window --title="$TITLE_PREFIX:$ns" -- \
                bash -lc "$env; $cmd 2>&1 | tee -a '$log'" >/dev/null 2>&1 || {
                    warn "gnome-terminal 启动失败，$ns 退回 log 模式"; MODE_RESOLVED=log; }
            ;;
        xterm)
            xterm -T "$TITLE_PREFIX:$ns" -e bash -lc \
                "$env; $cmd 2>&1 | tee -a '$log'" >/dev/null 2>&1 || {
                    warn "xterm 启动失败，$ns 退回 log 模式"; MODE_RESOLVED=log; }
            ;;
    esac

    if [[ "$MODE_RESOLVED" == log ]]; then
        setsid bash -lc "$env; $cmd" >> "$log" 2>&1 &
        printf '%s\n' "$!" > "$PID_DIR/$ns.pid"
    fi
    say "  $ns  rviz=$rviz  bridges=$(instance_bridges "$index")  log=$log"
}

# ---------------------------------------------------------------- stop
stop_ns() {
    local ns="$1" hit=0 pid pidfile
    pidfile="$PID_DIR/$ns.pid"
    if [[ -r "$pidfile" ]]; then
        pid="$(cat "$pidfile" 2>/dev/null || true)"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
            hit=1
        fi
        rm -f "$pidfile"
    fi
    if pgrep -f "simulation.launch.py.*robot_namespace:=$ns" >/dev/null 2>&1; then
        pkill -TERM -f "simulation.launch.py.*robot_namespace:=$ns" 2>/dev/null || true
        hit=1
    fi
    if [[ "$hit" -eq 1 ]]; then
        say "已停止 $ns"
    fi
}

stop_all() {
    local -a targets=()
    local ns any=0
    if [[ -r "$NS_FILE" ]]; then
        mapfile -t targets < "$NS_FILE"
    fi
    for ns in "${targets[@]+"${targets[@]}"}"; do
        [[ -z "$ns" ]] && continue
        stop_ns "$ns"
        any=1
    done
    if [[ "$any" -eq 0 ]]; then
        say "没有 $NS_FILE 记录的实例；如需停止请先用 --namespaces 指定，或直接 pkill"
    fi
    rm -f "$NS_FILE"
}

# ---------------------------------------------------------------- main
resolve_namespaces
# Resolve the terminal before sudo so DISPLAY/XAUTHORITY are exported into the
# re-executed (root) process; the root side cannot find the user session itself.
resolve_mode
maybe_sudo

if [[ "$ACTION" == stop ]]; then
    stop_all
    exit 0
fi

mkdir -p "$LOG_DIR" "$PID_DIR"

say "工作区     : $REPO"
say "ROS 域     : ${ROS_DOMAIN_ID:-0（未设置，默认 0）}"
say "实例       : ${NS_LIST[*]}"
say "终端模式   : $MODE_RESOLVED"
say "运行参数   : use_sim_time=$SIM_TIME yolo=$YOLO video=$VIDEO rviz_all=$RVIZ_ALL bridges_all=$BRIDGES_ALL"
if ((${#EXTRA_ARGS[@]})); then
    say "额外参数   : ${EXTRA_ARGS[*]}"
fi
preflight

if [[ "$DRY_RUN" == true ]]; then
    say ""
    say "预检完成，未启动。将执行："
    for i in "${!NS_LIST[@]}"; do
        say "  [${NS_LIST[$i]}] $(launch_cmd "${NS_LIST[$i]}" "$i" "$(instance_rviz "$i")")"
    done
    exit 0
fi

printf '%s\n' "${NS_LIST[@]}" > "$NS_FILE"

say ""
say "启动实例（仿真器需已在 sim_ws 中运行，且同一 ROS 域 / GPS 原点）："
for i in "${!NS_LIST[@]}"; do
    launch_instance "${NS_LIST[$i]}" "$i"
done

say ""
say "日志目录: $LOG_DIR"
if [[ "$FOLLOW" == true ]]; then
    say "tail -f 全部日志（Ctrl-C 只退出跟踪，不停止实例）"
    exec tail -n 20 -f "$LOG_DIR"/*.log
fi
say "跟踪日志: tail -f $LOG_DIR/*.log"
say "停止全部: bash scripts/dev/launch_multi_sim.sh --stop"
