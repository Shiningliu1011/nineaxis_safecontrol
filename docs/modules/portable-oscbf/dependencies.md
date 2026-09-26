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
| `tetgen` | 对经过检查的 closed collision mesh 执行离线四面体化 | ✅ 几何资产生成 |
| `jsonschema` | 检查版本化 CollisionParameterArtifact 与参数来源记录 | ✅ 参数加载 |

控制热路径固定使用 `cbfpy==0.0.1`、`jax==0.6.2`、`jaxlib==0.6.2`
和 `qpax==0.1.4`。版本清单见 `requirements.txt`，安装配置见仓库根目录
`setup.py`。

参数加载依赖 `jsonschema==4.23.0`，同样记录在运行依赖清单与 `setup.py` 中。加载完成后，JSON Schema 校验不进入控制周期。

离线几何资产生成依赖的固定版本见 `portable_oscbf/requirements-geometry.txt`。该文件用于生成与验证，不进入控制线程。
