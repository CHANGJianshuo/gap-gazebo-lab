> 本次规划演示的执行示例已在 scripts/demo.py 中实现。它不替代 SJM 后续正式抓放动作服务器。

# mtc_manipulation

负责人：SJM。状态：目录骨架，抓放动作服务器尚未实现。

实现靠近、抓取、夹爪闭合、抬升、转运、下放、释放、撤退的动作接口，封装轨迹执行、夹爪状态、失败码和取消。调用 motion_planning，不重复实现另一套高层 Task Manager。

预留 `src/`、`include/mtc_manipulation/`、`config/`、`launch/`、`test/`。建议需要 MoveIt 执行接口的部分用 C++。验收以反馈和物理接触下的稳定抓放为准；MoveIt attached collision object 不等价于仿真已夹住电池。
