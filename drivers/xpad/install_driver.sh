#!/usr/bin/env bash
# Install the locally built upstream xpad module for this exact Jetson kernel.
set -euo pipefail
cd "$(dirname "$0")"
if [[ "$EUID" -ne 0 ]]; then
  echo '请使用 sudo bash /home/a/ros2_ws/gz_sim/drivers/xpad/install_driver.sh'
  exit 1
fi
expected_kernel='5.15.148-tegra'
if [[ "$(uname -r)" != "$expected_kernel" ]]; then
  echo "内核版本不匹配，需要重新编译；期望 $expected_kernel"
  exit 1
fi
[[ -f xpad.ko ]]
[[ "$(modinfo -F vermagic ./xpad.ko)" == "$expected_kernel "* ]]
target_user="${SUDO_USER:-a}"
target_group="$(id -gn "$target_user")"
module_dir="/lib/modules/$expected_kernel/extra"
rule_file='/etc/udev/rules.d/71-gz-sim-xbox-controller.rules'
rule="SUBSYSTEM==\"input\", ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"028e\", GROUP=\"$target_group\", MODE=\"0660\", TAG+=\"uaccess\""
if [[ -f "$module_dir/xpad.ko" ]] && ! cmp -s xpad.ko "$module_dir/xpad.ko"; then
  echo "已有不同版本的 $module_dir/xpad.ko，未覆盖。"
  exit 1
fi
if [[ -f "$rule_file" ]] && [[ "$(cat "$rule_file")" != "$rule" ]]; then
  echo "已有不同的权限规则 $rule_file，未覆盖。"
  exit 1
fi
install -d "$module_dir"
install -m 0644 xpad.ko "$module_dir/xpad.ko"
printf '%s\n' "$rule" > "$rule_file"
chmod 0644 "$rule_file"
depmod -a "$expected_kernel"
udevadm control --reload-rules
modprobe joydev
modprobe xpad
udevadm trigger --subsystem-match=input --action=add
udevadm settle
printf '\n驱动已加载，当前手柄输入设备：\n'
find /dev/input -maxdepth 1 -name 'js*' -exec ls -l {} \;
printf '\n如果上面没有 js 设备，请拔插一次手柄接收器，然后告诉我执行结果。\n'
