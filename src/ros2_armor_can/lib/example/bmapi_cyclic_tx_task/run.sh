
# 设置库路径，适配 Linux
export LD_LIBRARY_PATH=.:../../bin/unix64/release:$LD_LIBRARY_PATH

# 推荐用当前 shell 直接运行
./bmapi_cyclic_tx_task 0 tx 1000 1

# 如果提示权限不足，再用 sudo -E 运行（-E 保证环境变量继承）
# sudo -E ./bmapi_cyclic_tx_task 0 tx 1000 1