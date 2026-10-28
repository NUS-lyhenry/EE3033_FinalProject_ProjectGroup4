# Agent runbook：本地 UI、ROS 后端与实车连接

更新日期：2026-10-27。本文按仓库现有启动脚本整理；路径、端口和在线状态需在每次运行时核对。文件名按要求为 `agent.md`，不要假定工具会像 `AGENTS.md` 一样自动加载它。

## 1. 先选运行方式

| 目的 | 启动入口 | 浏览器地址 | 后端与硬件 |
| --- | --- | --- | --- |
| 查看页面布局 | `scripts/preview_ui.py` | `http://127.0.0.1:8790/#/workflow` | 只读示例数据，无 ROS、无机器人控制 |
| 点击体验建图、搜索、返回 | `scripts/algorithm_preview_entrypoint.sh`，在专用 Docker 容器内运行 | `http://127.0.0.1:8792/#/workflow` | 项目 ROS 算法 + 模拟传感器和底盘 |
| 操作真实机器人 | 机器人容器内运行 `assessment.launch.py`，Mac 建立 SSH 转发 | `http://127.0.0.1:8765/#/workflow` | 实车 ROS、传感器和底盘 |

前端是原生 HTML/CSS/JavaScript，位于 `src/ee3033_project/web/`；实际运行不需要 npm 开发服务器。ROS Console 同时提供页面和 `/api/*`。`npm` 用于浏览器测试。

真实后端依赖 ROS 2 Humble 的 `rclpy`、`cv_bridge`、Nav2、SLAM Toolbox 等。Mac 上的普通 Python 环境不能替代这些依赖；完整本地算法流程使用下述 Linux Docker 容器。

## 2. 工作目录与启动前检查

本机仓库位置：

```bash
export GROUP4_REPO="/Users/lyhenry/Library/CloudStorage/OneDrive-NationalUniversityofSingapore/University/Y3S1/EE3033/Project/EE3033_FinalProject_ProjectGroup4"
cd "$GROUP4_REPO"
git status --short
git log -1 --oneline
lsof -nP -iTCP:8765 -iTCP:18765 -iTCP:8790 -iTCP:8792 -sTCP:LISTEN
```

换机器时修改 `GROUP4_REPO`。新开终端也需要重新设置该变量。

- 有未提交改动时保留它们，不执行 reset、clean 或强制 checkout。本文整理时工作区存在未提交的算法改动，因此“当前工作区”不等于远端 main。
- 端口已被占用时，先确认进程的启动命令和用途，再决定复用或停止。不要批量终止 Python、SSH、Docker 或 ROS 进程。
- 本文是操作说明；阅读或维护文档本身不代表授权启动仿真、机器人或运动测试。
- UI 文案保持英文；示例和模拟结果必须明确标注 simulated。

## 3. 只读 UI 预览：最快查看页面

在 Mac 的仓库根目录执行：

```bash
cd "$GROUP4_REPO"
python3 scripts/preview_ui.py --port 8790
```

打开 `http://127.0.0.1:8790/#/workflow`。直接查看任务设置可用 `http://127.0.0.1:8790/#/mission`。

这个入口只使用 Python 标准库，所有 POST 请求都会被拒绝。按钮禁用是预期行为，不能用它验证搜索算法、目标定位或实车连接。顶部 Connected 仅说明页面能访问当前服务；还要看 simulated 标记和机器人遥测。

修改 web 文件后刷新浏览器即可。终端按 `Ctrl+C` 停止。不要用直接打开 `index.html` 或普通静态文件服务器替代项目 API 服务来验证完整流程。

## 4. 可交互算法仿真：完整本地 UI + 后端

### 构建与启动

需要已安装且正在运行的 Docker。使用用户指定的 Docker runtime/context；不要为了查看只读页面自动启动 Colima 或 Docker Desktop。

```bash
cd "$GROUP4_REPO"
docker version
docker ps -a --filter name=ee3033-algorithm-preview
docker build -f scripts/Dockerfile.preview -t ee3033-algorithm-preview:local .

GROUP4_SOURCE_TAG="$(git rev-parse --short HEAD)"
if test -n "$(git status --porcelain)"; then
  GROUP4_SOURCE_TAG="${GROUP4_SOURCE_TAG}-dirty"
fi

docker run --name ee3033-algorithm-preview \
  -p 127.0.0.1:8792:8790 \
  --mount "type=bind,source=$GROUP4_REPO,target=/work,readonly" \
  -e PREVIEW_MAIN_COMMIT="$GROUP4_SOURCE_TAG" \
  ee3033-algorithm-preview:local
```

如果同名容器已经存在，先确认它是否属于当前任务。可用 `docker start -a ee3033-algorithm-preview` 重启原容器；不要直接删掉其中的地图和结果。

打开 `http://127.0.0.1:8792/#/workflow`。首次启动会编译包、下载并校验 YOLO 权重、准备 cup 图像素材，服务不会立即就绪。排查启动进度：

```bash
docker logs --tail 100 ee3033-algorithm-preview
```

### 实际执行了什么

- `scripts/Dockerfile.preview`：ROS Humble、Nav2、SLAM、视觉依赖；默认 `ROS_DOMAIN_ID=121`、`ROS_LOCALHOST_ONLY=1`。
- `scripts/algorithm_preview_entrypoint.sh`：在容器 `/tmp` 下编译 overlay、准备权重及素材，再启动算法预览。
- `scripts/run_algorithm_preview.py`：真实项目 Console 和受管的 mapping/mission 启动流程。
- `scripts/simulation_robot.py`：模拟激光、相机、里程计和底盘响应；实际算法节点处理这些数据。

容器内 Console 使用 8765，HTTP relay 使用 8790；Mac 的 8792 映射到容器的 8790。`--public-port` 用于 relay 的外部 Origin 配置和地址提示，不能单靠它改变容器监听的 8790。

仿真读取的是挂载的**当前工作区**。`PREVIEW_MAIN_COMMIT` 只是版本标签，不会 checkout main；重启旧容器也不会更新其创建时设置的标签。需要与某个提交严格对照时，使用该提交的独立、干净源码目录，并据此设置标签。

### 点击流程和限制

1. Start automatic mapping，等待实际建图、返回及保存。
2. 检查地图，选择 **cup**，设置 Return tolerance 和 Preferred precision，再确认地图准备任务。
3. 在地图上设置机器人真实模拟位置和朝向；AMCL 初始位姿不会把模拟底盘传送到该位置。
4. Start autonomous mission，观察搜索、定位、报告、返回及静止验证。
5. 下载 trial JSON；需要新一轮时使用 Rebuild map。

当前仿真场景的目标图像是 **cup**。选择 bottle 不会把场景自动换成瓶子。仿真保留实际算法流程和速度，运行时间不保证很短；它不能证明真实相机精度、平衡车动力学或实车测试通过。

### 修改代码、导出、停止

只读挂载允许容器读取本机修改，但正在运行的 Python 节点不会自动重新导入模块。改后端、launch、参数或资源文件后，停止并重新启动容器；入口会重建 overlay。修改 Dockerfile 或镜像依赖还需要重建镜像，并保留旧容器数据后再更换容器。

模拟地图、结果和素材位于容器 `/tmp/ee3033_sim/`，编译目录是 `/tmp/preview-build`、`/tmp/preview-install`。需要保留时，在本机先创建目标目录并执行：

```bash
mkdir -p "$GROUP4_REPO/results/algorithm-preview"
docker cp ee3033-algorithm-preview:/tmp/ee3033_sim/. \
  "$GROUP4_REPO/results/algorithm-preview/"
docker stop -t 25 ee3033-algorithm-preview
```

停止容器会保留其文件；删除容器会丢失未导出的容器文件。重启进程不等于恢复之前的内存状态。保持 Docker bridge 隔离，不添加 host networking、设备映射或实车 ROS domain。

## 5. 真实机器人：后端在机器人，浏览器在 Mac

### 已知部署位置

以下为之前实车会话确认过的位置，重连时需重新核对，不能视为当前在线证明。

| 项目 | 已知值 |
| --- | --- |
| SSH 用户、主机名、历史 IP | `balance`、`balance6`、`10.221.243.174` |
| ROS 容器 | `ros2_humble` |
| 机器人宿主机 workspace | `/home/balance/ros2_ws` |
| 容器内源码 | `/home/ros2_ws/group4_project` |
| 基础机器人 underlay | `/home/ros2_ws/install/setup.bash` |
| 项目 overlay | `/home/ros2_ws/group4_ws/install/setup.bash` |
| 权重 | `/home/ros2_ws/weights/yolo26n.pt` |
| 地图 / 记录 | `/home/ros2_ws/group4_maps` / `/home/ros2_ws/group4_results/assessment` |
| 实车 ROS 设置 | domain 6，`rmw_fastrtps_cpp`，`ROS_LOCALHOST_ONLY=0` |

使用交互式 SSH 登录或现有密钥。密码不写进本文、脚本、命令行、日志或仓库。IP 变化或超时时先确认当前网络与机器人地址。

### 查看已有服务，再决定是否启动

Mac 上：

```bash
export GROUP4_ROBOT="balance@10.221.243.174"
ssh "$GROUP4_ROBOT"
```

机器人宿主机：

```bash
docker ps --filter name=ros2_humble
docker exec -it ros2_humble bash
```

容器内先检查既有 launch、日志和 API。不要并行启动第二套 assessment、驱动或导航，也不要在正在测试时重启后端。先从 UI Stop robot，等取消导航并确认静止，再操作其所属进程。

### 构建与启动 assessment

以下在机器人 `ros2_humble` 容器内执行，适用于源码已正确部署、旧实例已停止的情况：

```bash
cd /home/ros2_ws/group4_project
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
bash scripts/build_robot.sh
source /home/ros2_ws/group4_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash scripts/check_environment.sh

ros2 launch ee3033_project assessment.launch.py \
  allow_motion:=true \
  port:=8765 \
  map_directory:=/home/ros2_ws/group4_maps \
  session_directory:=/home/ros2_ws/group4_results/assessment
```

`allow_motion:=true` 用于用户已授权的现场运动测试；只检查服务时使用 `allow_motion:=false`。启动 assessment 只创建 Console，加载阶段后才启动相应 ROS 栈；开始运动仍需操作 Start。单页 Start automatic mapping 可以同时完成加载阶段和开始建图。

`assessment.launch.py` 管理整个两阶段流程。`console.launch.py` 只启动独立 Console，默认不管理阶段，不能用它代替完整流程。不要另外手工启动 mapping/mission 与 assessment 所管理的阶段重叠。

`software_version` 可选参数应填写**实际部署版本**，不能用 Mac 最新提交冒充机器人当前代码。GitHub 合并、CI 通过、页面更新都不等于实车已部署。

### 保留现场配置

Console 会读取 session 目录已有的 `vision.yaml`；也可通过 `vision_config:=/absolute/path.yaml` 指定已验证的文件。保留现场相机几何、标定资料、地图和测试记录，不用仓库默认值覆盖它们。

之前实车使用过独立的 usb_cam 时间戳修复库：`/home/ros2_ws/group4_driver_fixes/usb_cam-0.8.1/fixed/lib/libusb_cam.so`。相关入口为 `scripts/build_camera_clock_fix.sh` 和 `hardware.launch.py`。正常启动不需要反复重建或替换正在工作的相机驱动。

### Mac 建立标准 SSH 转发

在另一个 Mac 终端运行并保持打开：

```bash
export GROUP4_ROBOT="balance@10.221.243.174"
ssh -N -T -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8765:127.0.0.1:8765 \
  "$GROUP4_ROBOT"
```

打开 `http://127.0.0.1:8765/#/workflow`。此方式的 UI 和 API 都由机器人 Console 提供；改 Mac 上的 web 文件不会改变它。需要可选旧标定页面时，另外转发机器人 8780 到本机 8780。

如果本机 8765 已被临时代理占用，可把上面转发的**本地端口**改为 18765。可在 `http://127.0.0.1:18765/api/state` 做只读检查；直接在该端口操作 UI 会受到 Console 的 Origin 检查限制。完整操作应使用与 Console 配置一致的 8765，或使用正确处理 Origin 的代理。

## 6. 本机 UI + 实车 API：历史临时代理

之前为了只刷新 Mac 上的 UI 而不重启实车，使用过：

```text
浏览器 127.0.0.1:8765
  → 本机 Python 代理：web 文件每次从本地仓库读取
  → /api/* 转发到 127.0.0.1:18765
  → SSH 隧道
  → 机器人 127.0.0.1:8765
```

本机临时文件为 `/private/tmp/group4-live-english-ui.py`，SSH ControlMaster socket 为 `/private/tmp/ee3033-robot-control.sock`。这些文件不在仓库，可能被清理；脚本还硬编码了本机源码路径及历史机器人地址，不能视为通用启动入口。

复用前先读脚本并检查已有进程。该脚本启动时要求 18765 已能访问机器人 API，还会通过指定 ControlMaster **取消同一 SSH 会话原来的 8765 转发**，随后自己监听 8765。缺少原转发或 socket 时可能直接退出，不能仅凭文件存在就重复启动。

如果确认临时脚本仍与当前机器一致、端口空闲且没有既有隧道，其原始配套方式为：

```bash
ssh -M -S /private/tmp/ee3033-robot-control.sock \
  -o ControlPersist=600 -o ExitOnForwardFailure=yes -fNT \
  -L 127.0.0.1:8765:127.0.0.1:8765 \
  -L 127.0.0.1:18765:127.0.0.1:8765 \
  balance@10.221.243.174
python3 /private/tmp/group4-live-english-ui.py
```

这条链路连接真实机器人，不是仿真。18765 隧道断开时，代理本身仍可能监听 8765，但 API 返回 502。只修改静态 UI 不会升级机器人算法；新 API 字段需要相应后端版本。临时脚本缺失时，优先使用上一节的标准转发方式。

## 7. 健康检查与常见问题

只读检查示例，按实际模式改端口；输出选定字段以免把会话 token 写入日志：

```bash
curl -fsS http://127.0.0.1:8765/api/state | python3 -c '
import json, sys
d = json.load(sys.stdin)
m = d.get("mission", {})
print({"simulated": d.get("simulated", False), "phase": d.get("phase"),
       "state": m.get("state"), "ready": m.get("ready"),
       "reason": m.get("readiness_reason"), "health": d.get("health"),
       "stage": d.get("stage")})
'
```

| 现象 | 检查重点 |
| --- | --- |
| 页面能打开，但控件禁用 | 只读 8790 预览、allow_motion、阶段是否加载、health、故障、是否已结束 trial |
| Connected 但无真实数据 | simulated 标记、传感器更新时间、阶段 running；仅 HTTP 可访问不代表机器人已连接 |
| 8765 API 返回 502 | 本机临时代理的上游 18765 / SSH 隧道是否还在 |
| Address already in use | 用 lsof 确认进程；区分标准 SSH 8765 转发与本机 8765 代理 |
| rclpy / cv_bridge 无法导入 | 是否在 ROS Linux 环境；是否 source 了 ROS、underlay 和项目 overlay |
| 修改后页面仍旧 | Console 启动时缓存 web 资源；构建后在停止状态重启 Console；本机临时代理和只读预览可刷新读取 |
| 仿真搜索不到 bottle | 当前素材是 cup；界面目标类别不会更换模拟场景 |
| 建图阶段无 YOLO/相机画面 | 阶段 1 默认不启动任务相机；阶段 2 再检查视觉就绪状态 |
| 新精度设置不可用 | 确认机器人已部署支持相应字段的后端；新前端不会自动更新算法进程 |

UI 控制请求使用 `X-Project-Token`、`X-Control-Generation` 和 Origin 校验。诊断优先使用 GET；不要绕过这些保护去自动发起运动。

在已 source 的 ROS 环境，可用 `ros2 node list`、`ros2 topic list` 检查节点与话题。任务阶段的定位主链路为：`/project/target_observations` → TargetFilter → `/project/target_estimate` → Console / Mission。地图显示与 accepted 应来自同一估计；记录位置、协方差和 estimate ID，不要另外生成 UI 专用融合结果。

## 8. 验证与收尾

修改算法时参考仓库的 `pytest -q`；浏览器测试入口为：

```bash
cd "$GROUP4_REPO"
npm ci
npx playwright install chromium
npm run test:ui
```

浏览器测试默认使用 8790；先确认该端口没有被其他用途占用。测试应使用只读预览或明确指定的测试环境。原生 ROS 回归入口是 `tests/run_humble_checks.sh` 和 CI 的 native-navigation；在隔离 Linux 环境运行，不能使用实车 domain 6 执行模拟测试。

实车停止顺序：UI Stop robot → 等待取消与静止 → 对所属 assessment 终端按 `Ctrl+C`，等待其子进程退出 → 必要时关闭对应 SSH 转发。不要用停止整个 `ros2_humble` 容器替代正常收尾。

最后向用户说明：使用哪个端口和模式、运行哪个源码版本、是否已部署实车、验证范围、仍在运行的服务，以及是否有需保留的地图/结果。编译或 CI 成功与真实运动测试结果分别报告。

## 9. 2026-10-28 实车建图与导航注意事项

- 机器人实测外形宽 `0.18 m`、前后长 `0.115 m`（含轮子）。按矩形居中估算，半对角线为 `0.106801 m`；再加 `0.01 m` 安全边距，规划 footprint 半径约 `0.116801 m`。未核实的是车体相对 `base` 中心的前后边界，因此这个居中外接圆估算不能替代 footprint 原点及边界核验；不要盲目缩小 footprint。
- 实车建图默认分辨率为 `0.05 m`；仿真明确保留 `0.0125 m`。Nav2 global costmap 的 static layer 应随地图分辨率 resize。启动时先核实实际地图分辨率及 costmap 行为，不要把仿真的精细分辨率设置误用为实车默认值。
- `LaserScan` 近回波会被屏蔽：默认半径为 `0.08 m`（8 cm），现场用户选择的 `0.20 m`（20 cm）应保留。屏蔽后 scan 读数可能是 `NaN`；对应位置进入地图后是未观测/unknown，不是地图读数本身为 `NaN`，也不代表 free。不要擅自改变现场半径或把未观测区域当作 free。
- 返回阶段曾因 mapper 漏掉 `navigate_home` 而失败。当前 home 修复仅有部署文件 SHA-256 和行为树检查证据；尚无我们独立确认的修复后回家运动通过记录。若再次出现返回失败，先检查实际部署版本及 mapper 阶段流程，不要只改变近回波屏蔽半径或缩小 footprint。
- 保留已有相机曝光、TF 和相机时钟修复配置；保留地图、`vision.yaml` 与校准资料。新代码必须部署到其所属 stage，读回部署文件并比对实际 SHA-256，再重载对应 stage；仓库源码或 Git 提交哈希不能代替部署文件核验。
- 开始导航前核实地图上的机器人朝向确实指向前方。不要在用户明确要求不仿真时启动模拟流程；物理测试反馈须标明证据范围。
- 操作员反馈“基本通过我的测试”属于人工测试报告，不等同于完整自动化通过或正式验收。

当前已核实的建图版本记录：`mapping5cm_20261028_dcb78f2c6f`；对应已部署文件 SHA-256：`528dd934176c440cc5e52ff7b2ae1eeefd8d2363d77abac6e0971211b8039305`。此哈希及行为树检查可确认部署文件和流程结构，不代表已完成修复后的回家运动验证。每次复用前仍应从目标机器人读回文件并重新比对哈希。

## 10. 2026-10-28 定位与姿态配置注意事项

- PR #8（`fbef8d5`）已合并，六项 CI 检查通过。操作员报告实车曾观察到 `COMPLETE` 且 `return_verified=true`；这是现场测试记录，不能扩展成完整独立复测或全面验收结论。
- 控制器姿态只信任 CRC 校验通过的新数据包，通过 `/project/body_attitude` 发布并用主机接收时间判断新鲜度。旧 `/robot_euler` 可能把缓存数值重新打上新时间戳，不能凭其频率或新时间戳当作新姿态。
- 实测车轴枢轴高度为 `0.033 m`，相机相对车轴前向偏移为 `-0.01 m`；相机离地高度换算到车轴枢轴时使用 `camera_height - 0.033 m`。固定模式沿用兼容几何；控制器模式用图像采集时刻的 roll/pitch 补偿相机姿态和枢轴到相机的杆臂。控制器模式必须先捕获有效静止参考；相机安装参数或枢轴改变后，该参考失效，需重新采集。
- `enable_ab_maneuver` 默认关闭，仅关闭主动横向 A–B 机动；正常目标经过检测仍参与融合。接近步进有 `0.10 m` 档位，精度检查阈值为 `0.025 m`；不要为提高进度直接打开横向机动。
- 目标采集预算默认 `90 s` 墙钟时间，包含观察与导航移动。已取消“精度改善不足 `10%` 就提前停止”的条件；达到目标精度才提前报告，否则使用预算继续采集。预算到期时保留同一地图下最佳有效估计及其原始时间戳/age，不要把旧估计刷新成新观测。
- 检测器 confidence 与位置 `position_confidence` 是不同量。后者是基于当前地图支持区域的模型后验质量，估计位置落在 `0.20 m` 半径内的概率，不是实测定位准确率；展示时应同时保留半径、估计 σ 和观测 age。
- `STOPPED` 与内部安全错误保持为停止/失败结果；只有目标结果满足流程要求并完成回家验证才可报告 `COMPLETE`。停止错误不能因已找到目标而被覆盖。
- 没有已保存姿态配置文件时，不要传空 `attitude_config:=` 给 ROS launch；省略该参数。仅在配置文件存在并已核验时传入实际路径。
