#!/usr/bin/env python3
"""
Booster K1 5v5 配置脚本
用于快速配置机器人参数：team_id, player_id, player_role, game_control_ip
"""

import os
import re
import sys
import subprocess

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# 配置文件路径 (相对于根目录)
CONFIG_FILE = os.path.join(script_dir, "src", "brain", "config", "config.yaml")
CONFIG_FILE_EXAMPLE = os.path.join(script_dir, "src", "brain", "config", "config.yaml.example")
GAME_CONTROL_LAUNCH_PY = os.path.join(script_dir, "src", "game_controller", "launch", "launch.py")

# 有效选项
VALID_PLAYER_IDS = [1, 2, 3, 4, 5]
VALID_ROLES = ["striker", "goal_keeper"]


def make_scripts_executable():
    """赋予 scripts 目录下所有文件可执行权限"""
    scripts_dir = os.path.join(script_dir, "scripts")
    if os.path.exists(scripts_dir):
        for filename in os.listdir(scripts_dir):
            filepath = os.path.join(scripts_dir, filename)
            if os.path.isfile(filepath):
                try:
                    os.chmod(filepath, 0o755)
                except Exception:
                    pass


def print_current_config(config_content):
    """显示当前配置"""
    print("-" * 60)
    print("当前配置:")

    # 提取相关配置行
    patterns = {
        "team_id": r"team_id:\s*(\S+)",
        "player_id": r"player_id:\s*(\S+)",
        "player_role": r'player_role:\s*"(\S+)"',
        "game_control_ip": r'game_control_ip:\s*"(\S+)"',
    }

    for key, pattern in patterns.items():
        match = re.search(pattern, config_content)
        if match:
            value = match.group(1)
            status = "✗ 待配置" if value in ["0", "TODO", '"TODO"'] else "✓"
            print(f"  {key}: {value:25s} [{status}]")

    # 读取 launch.py 中的 ip_white_list
    if os.path.exists(GAME_CONTROL_LAUNCH_PY):
        with open(GAME_CONTROL_LAUNCH_PY, "r", encoding="utf-8") as f:
            launch_content = f.read()
        ip_white_list_match = re.search(r'"ip_white_list":\s*\[([\s\S]*?)\]', launch_content)
        if ip_white_list_match:
            ip_list_raw = ip_white_list_match.group(1).strip()
            # 提取所有 IP 地址
            ips = re.findall(r'"([^"]+)"', ip_list_raw)
            if ips:
                ip_value = ', '.join(ips)
                status = "✗ 待配置" if "TODO" in ip_value else "✓"
                print(f"  ip_white_list:    {ip_value:25s} [{status}]")
            else:
                print(f"  ip_white_list:    (空)                      [✗ 待配置]")

    print("-" * 60)
    print()


def create_config_from_example():
    """如果 config.yaml 不存在，从 config.yaml.example 创建"""
    if not os.path.exists(CONFIG_FILE_EXAMPLE):
        print(f"❌ 错误：找不到配置文件模板：{CONFIG_FILE_EXAMPLE}")
        sys.exit(1)

    if not os.path.exists(CONFIG_FILE):
        print("未找到 config.yaml，正在从 config.yaml.example 创建...")
        import shutil
        shutil.copy2(CONFIG_FILE_EXAMPLE, CONFIG_FILE)
        print(f"✅ 已创建配置文件：{CONFIG_FILE}\n")
    else:
        print(f"✅ 配置文件已存在：{CONFIG_FILE}\n")


def input_with_validation(prompt, validator, error_msg):
    """带验证的输入函数"""
    while True:
        user_input = input(prompt).strip()
        if validator(user_input):
            return user_input
        print(f"  ❌ {error_msg}")


def get_team_id():
    """获取 team_id"""
    print("\n【步骤 1/4】设置队伍 ID (team_id)")
    print("  说明：必须与 GameControl 裁判系统的队伍 ID 保持一致")
    print("  有效范围：1-255")

    def validator(val):
        try:
            num = int(val)
            return 1 <= num <= 255
        except ValueError:
            return False

    return input_with_validation(
        "  请输入 team_id: ",
        validator,
        "无效的队伍 ID，请输入 1-255 之间的整数"
    )


def get_player_id():
    """获取 player_id"""
    print("\n【步骤 2/4】设置球员 ID (player_id)")
    print(f"  说明：机器人在队伍中的编号")
    print(f"  有效选项：{VALID_PLAYER_IDS}")

    def validator(val):
        try:
            num = int(val)
            return num in VALID_PLAYER_IDS
        except ValueError:
            return False

    return input_with_validation(
        "  请输入 player_id: ",
        validator,
        f"无效的球员 ID，请输入 {VALID_PLAYER_IDS} 中的一个数字"
    )


def get_player_role():
    """获取 player_role"""
    print("\n【步骤 3/4】设置球员角色 (player_role)")
    print(f"  说明：机器人在场上的角色")
    print(f"  有效选项：{VALID_ROLES}")
    print("    - striker: 前锋，负责进攻")
    print("    - goal_keeper: 守门员，负责防守球门")

    def validator(val):
        return val.lower() in VALID_ROLES

    return input_with_validation(
        "  请输入 player_role: ",
        validator,
        f"无效的角色，请输入 {' 或 '.join(VALID_ROLES)}"
    )


def get_game_control_ip():
    """获取 game_control_ip"""
    print("\n【步骤 4/4】设置裁判机 IP 地址 (game_control_ip)")
    print("  说明：运行 GameControl 裁判系统的机器 IP 地址")
    print("  格式：标准的 IPv4 地址，如 192.168.1.100")
    print("  提示：该 IP 将同时用于 ip_white_list")

    # 简单的 IP 地址验证
    def validator(val):
        # 允许 TODO 作为特殊情况
        if val == "TODO":
            return False
        # 简单的 IPv4 格式验证
        parts = val.split(".")
        if len(parts) != 4:
            return False
        try:
            return all(0 <= int(part) <= 255 for part in parts)
        except ValueError:
            return False

    return input_with_validation(
        "  请输入 game_control_ip: ",
        validator,
        "无效的 IP 地址，请输入格式如 192.168.1.100"
    )


def update_config_file(team_id, player_id, player_role, game_control_ip):
    """更新配置文件"""
    if not os.path.exists(CONFIG_FILE):
        print(f"\n❌ 错误：配置文件不存在：{CONFIG_FILE}")
        return False

    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        content = f.read()

    # 替换配置值
    replacements = [
        (r'(team_id:\s*)\S+', r'\g<1>' + team_id),
        (r'(player_id:\s*)\S+', r'\g<1>' + player_id),
        (r'(player_role:\s*)"\S+"', r'\g<1>"' + player_role + '"'),
        (r'(game_control_ip:\s*)"\S+"', r'\g<1>"' + game_control_ip + '"'),
    ]

    for pattern, replacement in replacements:
        content = re.sub(pattern, replacement, content)

    # 写回文件
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        f.write(content)

    return True


def update_launch_file(game_control_ip):
    """更新 launch.py 中的 ip_white_list，使用 game_control_ip"""
    if not os.path.exists(GAME_CONTROL_LAUNCH_PY):
        print(f"\n❌ 错误：launch 文件不存在：{GAME_CONTROL_LAUNCH_PY}")
        return False

    with open(GAME_CONTROL_LAUNCH_PY, "r", encoding="utf-8") as f:
        content = f.read()

    # 将 IP 转换为 Python 列表格式
    ip_list_str = f'"{game_control_ip}"'

    # 替换 ip_white_list 内容
    pattern = r'("ip_white_list":\s*\[)([\s\S]*?)(\])'
    replacement = r'\g<1>\n                        ' + ip_list_str + r'\n                        \g<3>'

    content = re.sub(pattern, replacement, content)

    # 写回文件
    with open(GAME_CONTROL_LAUNCH_PY, "w", encoding="utf-8") as f:
        f.write(content)

    return True


def main():
    # 首先赋予 scripts 目录下所有文件可执行权限
    make_scripts_executable()
    print("已赋予 scripts 目录下所有文件可执行权限\n")

    # 从 example 创建 config.yaml（如果不存在）
    create_config_from_example()

    # 读取当前配置
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        current_config = f.read()

    print_current_config(current_config)

    # 检查是否已经有有效配置
    has_todo = any(x in current_config for x in ["team_id: 0", "player_id: 0", '"TODO"', "game_control_ip: \"TODO\""])

    if not has_todo:
        print("⚠️  当前配置似乎已经完成。")
        print("   继续将覆盖现有配置。")
        cont = input("   是否继续？(y/n): ").strip().lower()
        if cont != 'y':
            print("已取消。")
            return

    print("\n请按照提示输入配置参数...")
    print()

    # 获取所有配置
    team_id = get_team_id()
    player_id = get_player_id()
    player_role = get_player_role()
    game_control_ip = get_game_control_ip()

    # 显示确认信息
    print("\n" + "=" * 60)
    print("请确认以下配置:")
    print("-" * 60)
    print(f"  team_id:          {team_id}")
    print(f"  player_id:        {player_id}")
    print(f"  player_role:      {player_role}")
    print(f"  game_control_ip:  {game_control_ip}")
    print(f"  ip_white_list:    {game_control_ip} (与 game_control_ip 相同)")
    print("-" * 60)

    confirm = input("确认保存配置？(y/n): ").strip().lower()
    if confirm != 'y':
        print("\n已取消，配置未保存。")
        return

    # 更新配置文件
    if update_config_file(team_id, player_id, player_role, game_control_ip):
        print("\n✅ 配置已成功保存!")
        print(f"   文件：{CONFIG_FILE}")
    else:
        print("\n❌ 配置文件保存失败!")
        sys.exit(1)

    # 更新 launch 文件（使用 game_control_ip 作为 ip_white_list）
    if update_launch_file(game_control_ip):
        print(f"   文件：{GAME_CONTROL_LAUNCH_PY}")
        print("\n提示：配置完成后，先colcon build，再运行 ./scripts/start.sh 启动机器人系统")
    else:
        print("\n❌ launch 文件保存失败!")
        sys.exit(1)


if __name__ == "__main__":
    main()
