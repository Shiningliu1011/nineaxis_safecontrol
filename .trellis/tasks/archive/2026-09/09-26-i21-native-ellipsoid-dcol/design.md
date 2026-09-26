# 技术设计

## 边界

`CollisionSafety.query` 接入原生 ellipsoid self query：内部几何加载与 DCOL 数值模块经 facade 提供公开操作，并由对应测试及接口文档记录。环境 point-scale、毫米距离、连续区间证明、生产切换继续由对应票处理。具有有效 environment support 的查询保留非 `OK`，同时提供 self 数值诊断。

`CollisionSafety` 构造时可绑定生成器格式的 geometry document；校验内容 hash、link identity、slot 数据、完整候选集合、允许接触记录和 policy。构造时固定 primitive identity、有效组合、margin 与容量，控制查询期间不读取文件。缺少 geometry 时保留现有未就绪行为。测试使用明确标识的解析 ellipsoid 数据，不声明 mesh 覆盖或生产参数资格。

## 数值方法

ellipsoid 写为 `c + L u`，`||u|| <= alpha`。令 `d=c2-c1`、`Q_i=L_i L_i^T`，两项凸二次约束的对偶为最大化 `t(1-t) d^T ((1-t)Q1+t Q2)^-1 d`，`0<=t<=1`。凹函数的一维驻点由有界二分求解；取平方根得到线性 DCOL scale。迭代上限和 residual 限制来自 parameter artifact。

最终 witness 同时检查两个 ellipsoid 原始 scaling inequality、线性求解 residual、对偶可行性与间隙。梯度使用最优值 envelope derivative，停止对已求得的 `t` 求导，随后通过共享 POE 运动学传播到九轴。中心重合处线性 scale 没有唯一梯度，返回零 scale 诊断和无效 solver health。

margin 使用 `1 + clearance_mm * 1e-3 / (min(radii_i)+min(radii_j))`。每个有效 pair 独立输出；不聚合梯度。kernel 使用 JIT 和批量运算，facade 在测量完成时间前等待 device 计算完成，超时使全部准入 mask 无效。

## 验证与风险

公开 `query` 是所有数值测试入口。解析球体与轴向 ellipsoid 给出精确答案；独立高精度原始 KKT 求解与官方 Julia conic solver 交叉核对一般姿态。差分覆盖旋转和位移。共享控制内核属于高风险范围，质量检查之后独立审查规范与需求。生产设备 deadline 和完整碰撞能力由执行地图后续验收负责。
