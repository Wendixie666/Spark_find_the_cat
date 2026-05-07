#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
机器人端脚本

用法:
  bash setup_robot_ros.sh \
    --mode multi \
    --robot-ip 192.168.1.101 \
    --robot-name spark \
    --host-ip 192.168.1.100 \
    --host-name laptop \
    --ros-distro noetic

模式:
  --mode multi
      ROS 多机连接模式
      机器人端:
        ROS_MASTER_URI -> 机器人自己的IP:11311
        ROS_IP         -> 机器人自己的IP

  --mode restore
      还原模式
      机器人端:
        ROS_MASTER_URI -> 机器人自己的IP:11311
        ROS_IP         -> 机器人自己的IP

说明:
  对于你这套拓扑，机器人端本来就是 roscore 所在端，
  所以 multi 和 restore 在机器人端的 ROS_MASTER_URI 结果一致，
  差别主要体现在“语义模式”上，便于两端统一操作。:contentReference[oaicite:1]{index=1}

参数:
  --mode         multi 或 restore
  --robot-ip     机器人端 IP
  --robot-name   机器人端 hostname
  --host-ip      主机端 IP
  --host-name    主机端 hostname
  --ros-distro   ROS 发行版，默认 noetic
EOF
}

MODE=""
ROBOT_IP=""
ROBOT_NAME=""
HOST_IP=""
HOST_NAME=""
ROS_DISTRO="noetic"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"; shift 2 ;;
    --robot-ip)
      ROBOT_IP="${2:-}"; shift 2 ;;
    --robot-name)
      ROBOT_NAME="${2:-}"; shift 2 ;;
    --host-ip)
      HOST_IP="${2:-}"; shift 2 ;;
    --host-name)
      HOST_NAME="${2:-}"; shift 2 ;;
    --ros-distro)
      ROS_DISTRO="${2:-}"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "未知参数: $1"
      usage
      exit 1 ;;
  esac
done

if [[ -z "$MODE" || -z "$ROBOT_IP" || -z "$ROBOT_NAME" || -z "$HOST_IP" || -z "$HOST_NAME" ]]; then
  echo "缺少必要参数"
  usage
  exit 1
fi

if [[ "$MODE" != "multi" && "$MODE" != "restore" ]]; then
  echo "--mode 只能是 multi 或 restore"
  exit 1
fi

BASHRC="${HOME}/.bashrc"
HOSTS_FILE="/etc/hosts"

BASHRC_BACKUP="${BASHRC}.bak.$(date +%Y%m%d%H%M%S)"
HOSTS_BACKUP="${HOSTS_FILE}.bak.$(date +%Y%m%d%H%M%S)"

cp "$BASHRC" "$BASHRC_BACKUP"
sudo cp "$HOSTS_FILE" "$HOSTS_BACKUP"

echo "已备份:"
echo "  $BASHRC_BACKUP"
echo "  $HOSTS_BACKUP"

ensure_line() {
  local pattern="$1"
  local newline="$2"
  local file="$3"

  if grep -Eq "^${pattern}$" "$file"; then
    sed -i "s|^${pattern}$|${newline}|" "$file"
  else
    printf '\n%s\n' "$newline" >> "$file"
  fi
}

ensure_hosts_entry() {
  local ip="$1"
  local hostname="$2"

  if grep -Eq "^[[:space:]]*${ip}[[:space:]]+${hostname}([[:space:]]|$)" "$HOSTS_FILE"; then
    return 0
  fi

  sudo sed -i "/[[:space:]]${hostname}\([[:space:]]\|$\)/d" "$HOSTS_FILE"
  echo "${ip} ${hostname}" | sudo tee -a "$HOSTS_FILE" >/dev/null
}

if ! grep -Eq "^source /opt/ros/${ROS_DISTRO}/setup\.bash$" "$BASHRC"; then
  printf '\nsource /opt/ros/%s/setup.bash\n' "$ROS_DISTRO" >> "$BASHRC"
fi

# 按你提供的拓扑，机器人端就是 roscore 所在端
MASTER_IP="$ROBOT_IP"
LOCAL_IP="$ROBOT_IP"

if [[ "$MODE" == "multi" ]]; then
  MODE_DESC="ROS 多机连接模式"
else
  MODE_DESC="还原模式"
fi

ensure_line 'export ROS_MASTER_URI=.*' "export ROS_MASTER_URI=http://${MASTER_IP}:11311" "$BASHRC"
ensure_line 'export ROS_IP=.*' "export ROS_IP=${LOCAL_IP}" "$BASHRC"

ensure_hosts_entry "$ROBOT_IP" "$ROBOT_NAME"
ensure_hosts_entry "$HOST_IP" "$HOST_NAME"

echo
echo "===== 机器人端配置完成 ====="
echo "模式: $MODE_DESC"
echo
echo "[~/.bashrc 关键项]"
grep -E 'source /opt/ros/|export ROS_MASTER_URI=|export ROS_IP=' "$BASHRC" || true

echo
echo "[/etc/hosts 关键项]"
grep -E "(${ROBOT_NAME}|${HOST_NAME})" "$HOSTS_FILE" || true

echo
echo "请执行:"
echo "  source ~/.bashrc"
echo
echo "如需启动 roscore，请执行:"
echo "  roscore"
