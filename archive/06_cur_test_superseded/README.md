# 06_cur_test_superseded
Moved out of `multiprocess_code/all_4_legs/cur_test/` because `robot/` supersedes it:
flat copies of the driver/runtime/homing, the old Y-up-frame runtime (`main_with_ik.py`,
`main_with_current_temp.py`, `inverse_kinematics.py`), old gait scripts (sidd/bound/pace/crawl/
daniel/trot_gait_node_sid), senders, bench logs and `extra_files/`.
They still run together from inside this folder (flat imports), but they use the OLD frame —
do not mix with `robot/`.
