## Overview

9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的安全控制项目。
当前仿真演示从随机位姿经 AEB-RRT* 过渡到蝴蝶轨迹起点，再由 OSCBF 控制器跟踪。
真机执行端处于 fail-closed containment：sim inert，shadow 仅记录且无 CAN I/O，
live 无条件拒绝。仿真验证不代表真实硬件能力或实机验收完成。
