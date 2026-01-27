'''
Please never modify this file directly.
'''

import socket
import json
import time
import threading
import numpy as np
from UR_server import URControllerProcess

class RobotVelocityController:    
    def __init__(self, server_ip="172.168.0.100", server_port=5005):
        self.UR_Proc = URControllerProcess(auto_start=False)
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.01)  # 10ms timeout
        self.server_address = (server_ip, server_port)
        self.latest_q = None
        self.latest_tcp_pose = None
        self.running = True
        print(f"Velocity controller initialized for {server_ip}:{server_port}")
    
    def send_velocity(self, qdot):
        try:
            if not isinstance(qdot, list):
                try:
                    qdot = qdot.tolist()
                except:
                    print("Error: qdot must be a list")
                    return False
            
            if len(qdot) != 6:
                print(f"Error: qdot must have 6 elements, got {len(qdot)}")
                return False
            
            message = json.dumps(qdot).encode('utf-8')
            self.sock.sendto(message, self.server_address)
            return True
        
        except Exception as e:
            print(f"Error sending velocity command: {e}")
            return False
    
    def receive_feedback(self):
        """Receive feedback data of q and tcp_pose from the server"""
        try:
            data, addr = self.sock.recvfrom(4096)
            feedback = json.loads(data.decode('utf-8'))
            
            if 'q' in feedback and 'tcp_pose' in feedback:
                self.latest_q = feedback['q']
                self.latest_tcp_pose = feedback['tcp_pose']
                return feedback
            return None
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Error receiving feedback: {e}")
            return None
    
    def read_data(self):
        while self.running:
            self.receive_feedback()
            # print("Receiving feedback...")
            time.sleep(0.002) # 2ms delay
    
    def stop(self):
        return self.send_velocity([0, 0, 0, 0, 0, 0])
    
    def close(self):
        self.UR_Proc.stop()
        self.stop()
        self.running = False
        self.thread.join()
        self.sock.close()
        print("Controller closed")
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_data)
        self.thread.start()
        self.send_velocity([0, 0, 0, 0, 0, 0])
        time.sleep(0.2)
    
    def get_current_q(self):
        """Get current joint angles"""
        return self.latest_q
    
    def get_current_tcp_pose(self):
        """Get current TCP pose"""
        return self.latest_tcp_pose
    
    def speedJ(self, qdot, duration=0):
        start_time = time.time()
        while True:
            self.send_velocity(qdot)
            # self.receive_feedback()
            time.sleep(0.008)
            if time.time() - start_time > duration:  # Run for the specified duration
                break

        if duration > 0:
            print("\nStopping robot...")
            self.stop()