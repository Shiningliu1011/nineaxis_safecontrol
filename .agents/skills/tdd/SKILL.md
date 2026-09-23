---
name: tdd
description: 用户要求测试先行、red-green-refactor 或集成测试，且当前 Trellis 任务存在可运行的公开接口测试入口时使用。
---

# Test-Driven Development

此项目采用 Matt Pocock `tdd` 的 red-green 循环。来源版本记录在 `.agents/skills/trellis-local/SKILL.md`。

读取任务需求、相关项目规范和现有测试。确定能够观察用户要求的公开接口，并在当前任务的 `implement.md` 记录测试边界与运行命令。按一个行为一个循环推进：编写有意义的失败测试，运行并记录失败结果，完成满足该行为的代码，再运行测试并记录通过结果。预期值应来自需求、已知样例或独立计算，不能由被测实现自行产生。

测试使用真实接口与项目允许的隔离环境，不使用 mock。需要仿真或真实设备才能观察的行为，遵守项目的安全边界。将 red/green 命令和结果写入 `implement.md`，随后执行受影响测试并交给 `trellis-check`。

此 skill 只在用户要求测试先行或集成测试时进入工作流；普通行为改动仍按项目的相关验证要求执行。
