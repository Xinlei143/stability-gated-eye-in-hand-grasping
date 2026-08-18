# phase

## R28. 日常开机运行

终端 1：

```bash
cd ~/robot_projects/foam_grasp_project
./scripts/setup_can.sh
timeout 5 candump -n 20 can0
./scripts/start_system.sh
```

终端 2：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
./scripts/check_system.sh
```

选择目标：

```bash
./启动.sh
```

或：

```bash
./启动.sh cube
./启动.sh cylinder
./启动.sh sphere
```


## R29. 标准停止与重新启动

抓取完成后让机械臂保持在安全位置。停止系统：

1. 在运行 `start_system.sh` 的终端按一次 `Ctrl+C`；
2. 等所有子进程 cleanly finished；
3. 检查没有核心残留；
4. 再关闭机械臂电源或按实验室规定失能。

```bash
pgrep -af \
  'piper_single_ctrl|piper_ctrl_single|move_group|orbbec|foam_' \
  || echo "核心进程已退出"
```

不要用关闭终端窗口代替正常 `Ctrl+C`。若所有终端误关：

```bash
cd ~/robot_projects/foam_grasp_project
source scripts/source_env.sh
ros2 node list
pgrep -af \
  'piper_single_ctrl|piper_ctrl_single|move_group|orbbec|foam_'
```

先确认实际进程状态，再决定是否重新启动。重复节点会导致 DDS 名称警告和错误
的命令/反馈连接。

