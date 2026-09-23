## 附录 B: 依赖清单

```
pip install jax jaxlib qpax cbfpy numpy scipy python-fcl trimesh pyyaml
# DCOL 碰撞内核: 依赖 DCOLuse/dpax 本地包 (可微碰撞), 见 DCOLuse/dpax_collision.py
# osqp 仅 legacy 参考, 非必需
```

---

> **维护规则**: 移植过程中每发现一个新的坑或调优经验，追加到移植项目的 LESSONS_LEARNED.md，保持与本项目一致的模板（现象/根因/修复/教训）。不得在本文档重新引入同一模块的第二个实现选择。
