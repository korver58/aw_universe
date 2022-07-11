# aw_universe
autoware universe build

colcon build の並列数を減らすとbuildできた。
https://zenn.dev/iwatake2222/scraps/1a8009020b8fe1

```bash
cd autoware
mkdir src
vcs import src < autoware.repos

```