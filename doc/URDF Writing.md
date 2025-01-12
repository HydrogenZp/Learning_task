# URDF Writing

## Link与Joint区别

- Link可见Joint不可见
- Joint=关节，链接两个及以上的Link
- 每个joint至多一个parent link，至少一个child link

## Xacro

- xacro为urdf提供模板（宏），方便参数修改与减少代码量
