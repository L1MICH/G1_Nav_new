# 测试死区值，用于标定

source /home/unitree/miniconda3/etc/profile.d/conda.sh
conda deactivate || true

python3 /home/unitree/wyh/new_nav/unitree_sdk2_python/example/g1/high_level/g1_deadzone_interactive_test.py eth0