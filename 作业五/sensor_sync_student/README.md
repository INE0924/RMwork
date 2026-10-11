# 第五次作业：并发传感器数据处理与时间同步

## 一、作业简介

在 RoboMaster 视觉系统中，相机和 IMU 的数据频率不同，也不一定同时到达。本项目使用两个文本文件模拟传感器数据流，利用 **C++17 多线程 + 有界线程安全队列** 收集数据，再为每一帧相机数据寻找时间戳最接近的 IMU 数据。

本题不需要 ROS 2、OpenCV 或真实摄像头。代码在参考包 `sensor_sync_student` 的框架上完成。

### 实现功能

- **两个生产者线程**：分别读取相机数据和 IMU 数据文件，把数据放进**同一个队列**。
- **一个消费者线程**：从队列中取数据，按 `CAMERA` / `IMU` 类型分别保存。
- **有界线程安全队列**：队列满时阻塞生产者；队列空时阻塞消费者；关闭队列后仍能取完剩余数据。
- **时间戳匹配**：排序后使用双指针寻找最近的 IMU，处理时间差并列、重复时间戳等情况。
- **异常处理**：配置或数据格式错误、文件无法打开时输出错误信息，以非零状态退出，避免线程卡死。

## 二、工程结构

```text
sensor_sync_student/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/
│   ├── thread_safe_queue.hpp    # 带容量限制的线程安全队列
│   └── sensor_sync.hpp          # 时间戳匹配算法
├── src/
│   └── main.cpp                 # 配置、生产者、消费者及输出
├── config/
│   └── sensor_sync.conf         # 程序配置
├── data/
│   ├── camera.txt               # 相机模拟数据
│   ├── imu.txt                  # IMU 模拟数据
│   └── expected_result.txt      # 公开样例的预期输出
├── docs/
│   └── packet_format.md         # 数据格式说明
├── verify_sample.sh             # 公开样例验证
└── tests/
    └── run_tests.sh             # 额外边界测试
```

## 三、编译与运行

环境：**Ubuntu 22.04、g++（支持 C++17）、CMake 3.16 或以上**。

安装基础工具（已经安装则不用重复执行）：

```bash
sudo apt update
sudo apt install -y build-essential cmake
```

在 Ubuntu 中进入项目根目录后执行：

```bash
cmake -S . -B build
cmake --build build -j
./build/sensor_sync config/sensor_sync.conf
cat data/result.txt
```

也可以直接运行参考包提供的验证脚本：

```bash
bash verify_sample.sh
```

额外测试（测试文件中的多种边界情况）：

```bash
bash tests/run_tests.sh
```

注意：`build/` 和 `data/result.txt` 是运行时生成的文件，不需要提交到 GitHub。

### 公开样例

相机数据 `data/camera.txt`：

```text
0 1000
1 2000
2 3000
3 5000
```

IMU 数据 `data/imu.txt`：

```text
0 800
1 1200
2 1900
3 3100
4 4200
```

预期输出 `data/result.txt`：

```text
CAMERA 0 IMU 0 DELTA 200
CAMERA 1 IMU 2 DELTA 100
CAMERA 2 IMU 3 DELTA 100
CAMERA 3 UNMATCHED
```

其中 `DELTA` 单位为**微秒（us）**，时间差等于阈值时也算匹配成功。

## 四、配置和输入说明

程序从命令行接收一个配置文件：

```bash
./build/sensor_sync config/sensor_sync.conf
```

配置示例：

```ini
queue_capacity=8
max_delta_us=300
camera_file=../data/camera.txt
imu_file=../data/imu.txt
output_file=../data/result.txt
```

- `queue_capacity`：队列最大容量，必须大于 0。
- `max_delta_us`：允许匹配的最大时间差，单位为微秒，必须大于等于 0。
- `camera_file`、`imu_file`：对应传感器数据文件。
- `output_file`：结果文件路径。
- 相对路径是以**配置文件所在目录**为基准，而不是命令执行目录。
- 配置允许空行和以 `#` 开头的注释；缺少必填项、重复键、未知键或非法数值会报错。

每行传感器数据格式：

```text
sequence timestamp_us
```

`sequence` 是非负 32 位整数，`timestamp_us` 是非负 64 位整数。单个文件内时间戳必须**非递减**。空行、负数、字段不足或多余字段均视为格式错误。具体格式见 `docs/packet_format.md`。

## 五、六个问题的回答

### 1. 队列已满或为空时，线程如何进入等待状态？

队列由 `std::mutex` 保护，并使用两个 `std::condition_variable`：`not_full_` 和 `not_empty_`。生产者调用 `push()` 时，如果队列已满，就使用 `wait()` 等待有空位；消费者调用 `pop()` 时，如果队列为空，就等待新数据。`wait()` 会暂时释放互斥锁，让其他线程能够继续操作队列；被唤醒后重新获取锁并检查条件。这比不断循环检查队列状态更节省 CPU。队列关闭也会唤醒所有等待线程。

### 2. 如何判断最后一个生产者已经结束？

程序使用 `std::atomic<int> remaining_producers{2}` 记录尚未结束的生产者数。每个生产者退出前执行一次 `fetch_sub(1)`，如果减之前的值为 1，说明自己是最后一个结束的生产者，这时调用 `queue.close()`。无论正常读完还是捕获异常，都会执行这一步。如果某个线程发生异常，程序还会提前关闭队列，避免其他线程一直等待。

### 3. 为什么关闭队列后仍然需要允许消费者读取剩余数据？

关闭队列只表示**不会再有新数据写入**，不代表原来保存的数据已经处理完。如果一关闭就让消费者退出，队列中尚未读取的传感器数据就会丢失，后面的时间戳匹配结果也可能不完整。因此 `pop()` 在队列关闭后仍能取出已有数据，只有**队列已关闭且内部已经为空**时才返回 `false`。

### 4. 时间戳匹配算法的具体流程和复杂度是什么？

消费者先把两路数据分别收集起来。所有线程结束后，按照**时间戳升序、序号升序**对相机和 IMU 数据排序。对于相同时间戳的多条 IMU，只保留序号最小的一条作为匹配候选。之后用一个指针沿着 IMU 时间戳向前移动：对每个相机包，只需比较它前后相邻的两个 IMU 时间戳，就能找到距离最小的候选。若最小时间差不超过 `max_delta_us`，输出匹配结果，否则输出 `UNMATCHED`。

设相机数据有 `C` 条、IMU 数据有 `I` 条。排序复杂度为 `O(C log C + I log I)`；去重和双指针扫描为 `O(C + I)`，所以总复杂度是 **`O(C log C + I log I + C + I)`**，没有对每一帧相机数据遍历全部 IMU。辅助空间复杂度为 `O(C + I)`。

### 5. 如何处理时间差相同、时间戳相同等边界情况？

- 两个候选 IMU 的**时间差相同**：优先选择时间戳更早的那一条。
- 多条 IMU 的**时间戳相同**：先按序号排序，优先选择序号更小的那一条。
- 多条相机数据的**时间戳相同**：输出时按相机序号从小到大排列。
- 相机时间戳和 IMU 时间戳完全相同：时间差为 0；即使 `max_delta_us=0` 也匹配成功。
- 没有 IMU 数据，或最近时间差大于阈值：输出 `UNMATCHED`。
- 同一条 IMU 可以匹配多帧相机数据，不会在匹配后删除。
- 时间戳用 `uint64_t` 表示，并通过大小比较后相减计算时间差，避免直接相减可能带来的无符号溢出问题。

### 6. 如果某个生产者读取文件失败，如何保证其他线程能够正常退出？

生产者用 `try/catch` 捕获无法打开文件、数据格式不正确等异常，把异常保存下来，并调用 `queue.close()`。关闭队列会唤醒所有等待在 `push()` 或 `pop()` 上的线程：另一个生产者发现队列已关闭后停止入队，消费者则继续处理队列里已经存在的数据，直到取空后退出。主线程使用 `join()` 等待三个线程全部结束，最后报告错误并返回非零退出码，不会把这次失败当作匹配成功。消费者自身发生异常时也会关闭队列，避免生产者因队列已满而死锁。

## 六、测试结论

在 C++17 编译环境下，已经通过参考包公开样例，以及队列容量为 1、时间差并列、重复时间戳、零阈值、64 位时间戳边界、空 IMU 文件、非法输入、缺失文件和并发压力等测试。

实际测试可以使用：

```bash
bash verify_sample.sh
bash tests/run_tests.sh
```
