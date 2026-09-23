## 8. 配置体系

```
portable_oscbf/config/
├── nineaxis.yaml          # 主配置: 关节限位/速度/控制器增益/路径/动态障碍物/QP
├── actuator_modules.yaml  # 电机规格
├── controller_params.yaml # 控制器增益
├── nullspace.yaml         # 零空间策略 (manipulability_gradient + 激活/滤波参数)
├── obb_model.yaml         # OBB 包络盒标定数据 (每连杆半边长/局部旋转)
├── fcl_params.yaml        # FCL 碰撞参数 (环境)
├── obstacle_params.yaml   # 障碍物场景
├── ompl_params.yaml       # 规划参数 (预留)
└── robot_params.yaml      # 机器人物理参数
```

控制热路径**不读 YAML**；参数在 `JaxControlLoop` 初始化时一次加载到配置对象，避免逐步 I/O。零空间策略经配置切换（`joint_center` ↔ `manipulability_gradient`），切换不修改控制循环和 QP。

---
