import rtde_control
import rtde_receive
import socket
import json
import time
import math

# UDP Server Configuration
UDP_IP = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 5005     # 监听端口

# Robot Configuration
ROBOT_IP = "192.168.50.168"
acceleration = 40
dt = 1.0/125  # 8ms

# Client Connection Management
CLIENT_TIMEOUT = 2.0  # 客户端超时时间（秒），超过此时间未收到消息则释放连接
active_client = None  # 当前活跃客户端地址
last_received_time = None  # 上次接收消息的时间

# 初始化机械臂控制接口
print(f"Connecting to robot at {ROBOT_IP}...")
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("Robot connected successfully")

# 创建UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.01)  # 设置10ms超时，保持控制循环频率

print(f"UDP Server listening on {UDP_IP}:{UDP_PORT}")
print("Waiting for velocity commands...")
print("Expected format: [q1, q2, q3, q4, q5, q6] (JSON array)")

qdot = [0, 0, 0, 0, 0, 0] 
q_range = [-math.pi, math.pi]  # Joint velocity limits

try:
    while True:
        t_start = rtde_c.initPeriod()
        current_time = time.time()
        
        # 检查当前客户端是否超时
        if active_client is not None and last_received_time is not None:
            if current_time - last_received_time > CLIENT_TIMEOUT:
                print(f"Client {active_client} timed out, releasing connection")
                active_client = None
                last_received_time = None
                qdot = [0, 0, 0, 0, 0, 0]  # 重置速度为0
        
        # 尝试接收UDP数据
        try:
            data, addr = sock.recvfrom(1024)
            
            # 检查是否为活跃客户端或没有活跃客户端
            if active_client is None:
                # 接受新客户端连接
                active_client = addr
                print(f"New client connected: {addr}")
            elif active_client != addr:
                # 拒绝其他客户端
                print(f"Rejected connection from {addr} - Client {active_client} is already connected")
                continue
            
            # 处理来自活跃客户端的数据
            received_qdot = json.loads(data.decode('utf-8'))
            
            # 验证数据格式
            if isinstance(received_qdot, list) and len(received_qdot) == 6:
                qdot = received_qdot
                last_received_time = current_time
                print(f"Received from {addr}: {qdot}")
            else:
                print(f"Invalid data format from {addr}: {data}")
        except socket.timeout:
            # 超时则继续使用上一次的速度指令
            pass
        except json.JSONDecodeError:
            print(f"JSON decode error: {data}")
        except Exception as e:
            pass
            # print(f"Error receiving data: {e}")
        
        # 执行速度控制
        
        qdot = [max(q_range[0], min(q_range[1], v)) for v in qdot]
        rtde_c.speedJ(qdot, acceleration, dt)
        
        # 如果有活跃客户端，发送当前状态
        if active_client is not None:
            try:
                q_current = rtde_r.getActualQ()
                tcp_pose = rtde_r.getActualTCPPose()
                
                # 构建响应数据
                response_data = {
                    "q": q_current,
                    "tcp_pose": tcp_pose
                }
                
                # 发送数据给客户端
                response_json = json.dumps(response_data)
                sock.sendto(response_json.encode('utf-8'), active_client)
            except Exception as e:
                print(f"Error sending feedback to client: {e}")
        
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    print("\nReceived interrupt, stopping robot...")
finally:
    # 停止机械臂
    print("Stopping robot...")
    rtde_c.speedStop()
    rtde_c.stopScript()
    sock.close()
    print("Server shutdown complete")