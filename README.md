# real_sim_retarget

iPhone 三维重建 + 双手动作捕捉，重定向（retarget）到双臂 Panda 机械臂 + 平行夹爪的 MuJoCo 仿真。

**目标流水线**：

```
iPhone 扫描/录像 → 三维场景重建 → 双手 21 关键点追踪
                                        │
                                        ▼
                     手部位姿 → 双臂夹爪目标位姿映射（retarget）
                                        │
                                        ▼
              MuJoCo 双臂 Panda + 平行夹爪场景复现操作过程
```

在仿真里先用夹爪近似复现人手的双手操作，后续可换灵巧手。

## 当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| R0 | 场景拼装器：从 yaml 配置生成双臂 Panda + 夹爪 + 物体的 MJCF 场景 | ✅ |
| R1 | iPhone 重建数据接入与坐标系对齐 | ⬜ |
| R2 | 双手关键点追踪与 retarget 映射 | ⬜ |
| R3 | 双臂闭环控制与环境封装（Gymnasium） | ⬜ |

R0 已实现的关键设计（`retarget/scene_builder.py`）：

- **双臂实例化**：Menagerie Panda 臂 + 平行夹爪，左右实例加 `l_`/`r_` 前缀防重名，motor 执行器改写为 position 并配 kp/forcerange
- **物体资产内联**：MJCF 的 `<include>` 不能改被引用文件的 body 属性，故解析 `object.xml` 后把 asset 片段（材质/mesh 加前缀）与 worldbody 内联进场景
- **16 维控制接口**：`[l_arm×7, l_gripper, r_arm×7, r_gripper]`，含无碰撞 home keyframe
- **生成即自检**：`gen_scene.py` 输出后立即加载校验 nq/nv/nu 与执行器顺序

## 快速开始

```bash
pip install mujoco pyyaml
python scripts/gen_scene.py   # 读取 configs/scene_bimanual.yaml，生成并自检场景
```

## 目录结构

```
real_sim_retarget/
├── assets/
│   ├── robot/            # Panda 臂 + 平行夹爪（含 meshes）
│   ├── objects/          # 物体资产（object.xml，底部中心为原点）
│   └── scenes/           # 生成的场景 XML
├── configs/              # 场景配置（物体列表 + 摆位）
├── retarget/             # 场景拼装器（R0）；后续放 retarget 映射
├── scripts/              # gen_scene.py 等工具脚本
├── teleop/               # （R2）双手追踪与映射
├── envs/                 # （R3）Gymnasium 环境
└── utils/
```

## 参考

- [MuJoCo](https://github.com/google-deepmind/mujoco) / [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
