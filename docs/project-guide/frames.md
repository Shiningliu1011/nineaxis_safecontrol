## 坐标系

- 规范世界坐标系为固定的 `base_link`，使用 Y-up；J1 沿该系 Z 轴移动，基座坐标系不随 J1 移动。
- MuJoCo 显示使用 Z-up，通过 `display_frame` body 的 euler 旋转转换。
- 坐标系决议见 [base_link 规范世界坐标系](../adr/0002-base-link-canonical-world-frame.md)。
