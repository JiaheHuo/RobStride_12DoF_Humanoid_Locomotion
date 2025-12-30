# RobStride USB2CAN Control

## 构建（含 libtorch）
1. 下载与编译环境一致的 [LibTorch](https://pytorch.org/get-started/locally/) 发行版并解压，例如 `/opt/libtorch`.
2. 配置 CMake 前缀（任选其一）：
   - `export CMAKE_PREFIX_PATH=/opt/libtorch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}`
   - 或在配置时传入 `-DTorch_DIR=/opt/libtorch/share/cmake/Torch`
   - 如果放在 `/opt/libtorch` 并保持默认路径，`CMakeLists.txt` 会自动尝试该 Torch_DIR。
3. 项目已在 `CMakeLists.txt` 中使用 `find_package(Torch REQUIRED)` 并链接 `${TORCH_LIBRARIES}`；无需手改源码，只需保证 CMake 能找到 libtorch。
4. 常见的本地构建命令：
   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ```

## 运行示例
- 传统测试：`./motor_test --config ../config/motors.yaml --test sine`
- 读取 TorchScript 策略（humanoid-gym 风格观测堆叠）：
  ```bash
  ./motor_test --config ../config/motors.yaml \
               --policy policy_script.pt \
               --policy-device cpu \
               --obs-stack 4 \
               --action-scale 0.15 \
               --action-clip 0.6
  ```

## 观测与策略
- 观测构成为：关节位姿/速度、原始电机状态、下发指令、IMU（acc/gyro/rpy），按 humanoid-gym 的方式在时间维度堆叠最近 `obs_stack` 帧（默认 4 帧，未填满时自动用零补齐）。
- 策略通过 libtorch 载入 TorchScript，输入 shape 为 `[1, obs_dim*obs_stack]` 的 `float32` 张量，输出 action（一维向量，自动压缩 batch 维）。
- `action-scale`：每个动作值会被 clamp 到 `[-1,1]` 后乘以该系数作为目标增量（rad）。`action-clip` 用于最终目标的幅值限制，并结合关节安全区间做二次裁剪。

## 安全限制（MotorLimit）
- 12 个下肢关节限制（rad）：  
  - Left1~6: `[-0.1,0.8]`, `[-0.3,0.1]`, `[-0.9,0.9]`, `[-1.57,0]`, `[-0.1,0.5]`, `[-0.5,0.1]`  
  - Right1~6: `[-0.8,0.1]`, `[-0.1,0.3]`, `[-0.9,0.9]`, `[0,1.57]`, `[-0.1,0.5]`, `[-0.5,0.1]`
- 一旦任意关节越界，系统立刻给所有电机下发阻尼（`kp=0, kd=YAML`），以 0 速度目标快速刹停。
