## 外部依赖

| 依赖 | 用途 | 必需? |
|------|------|-------|
| `jax` | 自动微分, JIT 编译, GPU 加速 | ✅ 核心 |
| `qpax` | JAX 可微分 QP 求解器 | ✅ 核心 |
| `cbfpy` | CBF 框架 (CBFConfig, CBF 类) | ✅ 核心 |
| `numpy` | 数组操作 | ✅ 核心 |
| `scipy` | 旋转, 稀疏矩阵, .mat 加载 | ✅ 核心 |
| `python-fcl` | FCL 距离/碰撞查询 | ✅ 碰撞检测 |
| `trimesh` | STL 网格加载 | ✅ 碰撞检测 |
| `osqp` | OSQP QP 求解器 (legacy 路径) | ⚠️ 可选 |
| `PyYAML` | YAML 配置加载 | ✅ 配置 |
| `urdfdom_py` (`urdf-parser-py`) | 解析 URDF 并生成运动学常量 | ✅ 生成与一致性测试 |

控制热路径固定使用 `cbfpy==0.0.1`、`jax==0.6.2`、`jaxlib==0.6.2`
和 `qpax==0.1.4`。版本清单见 `requirements.txt`，安装配置见仓库根目录
`setup.py`。
