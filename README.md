# 重构项目
## 重构思路
我打算做一个自动生成ros2和c的代码的工具
总的来说, 就是改为由yaml驱动的ROS和C的代码自动生成, 从而避免手动修改代码带来的风险, 也更加方便

protocol的功能优化, 定义放到protocol.yaml中

使用scripts中的codegen.py生成ros2和c的代码

我现在有现成的代码, 生成的ros2的代码应该一起放到src中, 与现有的代码融合

电控的c代码生成之后放到mcu_output中


## TODO
### 功能

- 默认心跳包检测通信是否正常
- 启动进行握手对比上下位机版本(通过build的时候生成一个Hash值)是否一致

#### 核心YAML配置

```yaml
# 1. 全局配置
protocol_settings:
  head_byte_A: 0xAA      # 帧头 A
  head_byte_B: 0x55      # 帧头 B
  checksum: "CRC8"     # 校验算法

# 2. 类型映射：定义协议类型缩写与 C 语言类型的映射关系
type_mappings:
  f32: "float"
  i32: "int32_t"
  u8:  "uint8_t"
  bool: "uint8_t"  # 1 byte

# 3. 消息列表(核心配置区域)
messages:

  ##################################默认检测心跳包#########################################

  # ROS -> MCU: 心跳请求
  - name: "HeartbeatRequest"
    id: 0x01                        
    direction: "tx"
    ros_msg: "std_msgs/msg/UInt8"   
    topic: "/heartbeat_tx"
    fields:
      - { proto: "count", type: "u8", ros: "data" }

  # MCU -> ROS: 心跳回应
  - name: "HeartbeatResponse"
    id: 0x02                        
    direction: "rx"
    ros_msg: "std_msgs/msg/UInt8"
    topic: "/heartbeat_rx"
    fields:
      - { proto: "count", type: "u8", ros: "data" }

  ###############################################################################

  # proto 是下位机代码里的变量名，而 ros 是ROS2消息包里的数据路径。
  # ROS -> MCU: 速度控制
  - name: "CmdVel"
    id: 0x03
    direction: "tx"                 
    ros_msg: "geometry_msgs/msg/Twist"
    topic: "/cmd_vel"
    fields:
      - { proto: "linear_x",  type: "f32", ros: "linear.x" }
      - { proto: "angular_z", type: "f32", ros: "angular.z" }


```

### 数据帧

波特率: 921600

- 双帧头
- 校验和使用CRC8(查表法)(校验办法可选)
- 开启TLV功能(可选)
- 去掉帧尾
        +Header1 : uint8 (0x5A)
        +Header2 : uint8 (0xA5)
        +ID      : uint8
        +Length  : uint8
        +Data    : uint8[Length]
        +CRC     : uint8/16 (Does check on ID+Len+Data)


### 代码层
- 避免内存拷贝, 直接在驱动提供的原始 buffer 上进行解析
- 提升串口通信的线程优先级



# 提醒

- 当前结构体最大字节255, 若需要传输更多数据, 需要修改数据定义为uint16_t
- 只支持固定长度的数据类型



### 性能优化
#### 1. 
使用了 std::deque<uint8_t> rx_buffer_。频繁内存拷贝: 
feed_data函数使用了 rx_buffer_.insert(...)，这会将接收到的数据拷贝进 deque。
低效的擦除: 
parse_packet函数解析成功后调用 rx_buffer_.erase(...)。虽然 deque 头删效率尚可，但在高吞吐量下，频繁的内存分配和释放仍是负担。
优化建议: 建议在单片机或高性能场景改用 环形缓冲区 (Ring Buffer)。

使用固定大小的 std::array 或 std::vector 配合读写指针 read_ptr / write_ptr。
消除 insert 和 erase 的开销，变为纯指针/索引操作。  


#### 2. 
在处理错位数据时，可以使用 std::find 快速跳过无效数据，直接定位到下一个 kHeadByte，一次性丢弃这段无效数据。

auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), kHeadByte);
if (it != rx_buffer_.begin()) {
    rx_buffer_.erase(rx_buffer_.begin(), it);
    return false; // 等待下一次处理
}



#### 3. 
优化建议: 修改 PacketHandler::feed_data 接口，使其接受指针和长度

// 优化后的接口
void feed_data(const uint8_t* data, size_t len);
// 调用处
packet_handler_.feed_data(buffer.data(), bytes_read);
这样可以实现零拷贝 (Zero-copy) 将数据喂入处理逻辑（当然 
feed_data
 内部存入 buffer 还是要拷一次，但省去了中间环节）。