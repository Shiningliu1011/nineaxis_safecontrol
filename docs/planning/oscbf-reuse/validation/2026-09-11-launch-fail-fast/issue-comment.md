Documentation / evidence follow-up

- c3b3f186: documentation reconciliation
- f16cc35c: containment validation + first Retro archive
- 本轮 source-of-truth cleanup 尚未提交：修正 README 验证版本口径、CONTEXT 状态流/真机目标定义及 controller 注释。

此前产品代码 containment 完整验收仍绑定 e4d9a268；上述后续提交和本轮文档/注释 diff 没有改变 hardware_bridge 的运行行为。launch-level fail-fast 将在本轮后续独立实施、验证，不能把历史验收当作该改动的证据。

Issue remains OPEN.
Containment != live implementation.
Containment != real-hardware acceptance.
