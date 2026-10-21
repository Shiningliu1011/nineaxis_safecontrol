# 完整周期计时集成验证

核心提交 `8d022236db92376cf1626fc24929b23ec86efd0e` 经独立规范与规格审查，
与前置检查提交 `a6a422a5f8dab264e0796a6faa771e9dbfc90a24` 普通合并至
`3771bc88deb14e38bc9c24624b2c881717af7190`，已推送集成分支。

124 项运行来源与定向验证源码完全一致，规范化来源映射 SHA-256 为
`1ccfd1fbb534b70f091e7e5874cd1d05b7ed60730fda2abbb5a7adb79b04109f`。
独立核查文件为主仓库
`.scratch/integration-frontier/complete-cycle-runtime-3771bc8.json`，SHA-256 为
`c89c496d1157b36a253978bbb895c7e483158b61cfb9f2ff1d469d6db6524dac`。
定向实际结果及数据位置见 `complete-cycle-timing-implementation.md`。

三卷前置入口的准备检查已经退出0，没有启动 sim 或创建实验目录。
计划文件为 `.scratch/static-implementation/complete-cycle-preflight-prepared.json`，
SHA-256 为 `8c42af5f174c118a413241d45fb151e7b36fb6a26a9ae6eeb9bf59031d90acaa`。
两个 NTFS 卷均按既有 UUID 正常读写挂载，原始数据保留。

集成完整回归由 `static-final-regression-04.service` 执行，日志为
`output/static-capacity-20261020/final-regression-04.log`，产物目录为
`.scratch/regression-xiLHz4Py/`。尚未取得最终结果，不能记为通过。

正式容量绑定的公共参数检查正在独立修复和验证，完成后需重新固定当前源码身份。
三卷前置、当前容量和共同周期全部通过后方可启动正式采集；当前为0/1080。
