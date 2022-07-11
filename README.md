# aw_universe
autoware universe build

colcon build の並列数を減らすとbuildできた。

```bash
cd autoware
mkdir src
vcs import src < autoware.repos

```