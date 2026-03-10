'''
Please never modify this file directly.
'''

from collections import deque
import socket
import json
import time
import threading
import numpy as np
from UR_server import URControllerProcess
import matplotlib.pyplot as plt

class RobotVelocityController:    
    def __init__(self, server_ip="172.168.0.100", server_port=5005, direct_connect=False):
        self.UR_Proc = URControllerProcess(auto_start=direct_connect)
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.01)  # 10ms timeout
        self.server_address = (server_ip, server_port)
        self.latest_q = None
        self.latest_tcp_pose = None
        self.running = True
        self.last_ctrl_t = 0.0
        print(f"Velocity controller initialized for {server_ip}:{server_port}")
    
    def send_velocity(self, velocity, mode="speedJ"):
        try:
            if not isinstance(velocity, list):
                try:
                    velocity = velocity.tolist()
                except:
                    print("Error: velocity must be a list")
                    return False
            
            if len(velocity) != 6:
                print(f"Error: velocity must have 6 elements, got {len(velocity)}")
                return False

            if mode == "speedJ":
                # Keep legacy packet format for compatibility.
                payload = velocity
            elif mode == "speedL":
                payload = {"mode": "speedL", "velocity": velocity}
            else:
                print(f"Error: unsupported mode '{mode}', expected 'speedJ' or 'speedL'")
                return False

            message = json.dumps(payload).encode('utf-8')
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
        return self.send_velocity([0, 0, 0, 0, 0, 0], mode="speedJ")
    
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
            self.send_velocity(qdot, mode="speedJ")
            # self.receive_feedback()
            time.sleep(0.008)
            if time.time() - start_time > duration:  # Run for the specified duration
                break

        if duration > 0:
            print("\nStopping robot...")
            self.stop()

    def speedL(self, xdot, duration=0):
        """Send Cartesian TCP velocity [vx, vy, vz, rx, ry, rz]."""
        start_time = time.time()
        while True:
            self.send_velocity(xdot, mode="speedL")
            time.sleep(0.008)
            if time.time() - start_time > duration:  # Run for the specified duration
                break

        if duration > 0:
            print("\nStopping robot...")
            self.stop()
    
    def draw_trajectory(self, target_speed, target_pos, t):

        # Lazy init for interactive plotting (keeps state across calls)
        if not hasattr(self, "_traj_init"):
            plt.ion()
            self._traj_t = deque(maxlen=2000)
            self._traj_pos = [deque(maxlen=2000) for _ in range(6)]
            self._traj_spd = [deque(maxlen=2000) for _ in range(6)]

            self._traj_fig, (self._traj_ax1, self._traj_ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
            self._traj_line_pos = []
            self._traj_line_spd = []
            colors = ['b', 'g', 'r', 'c', 'm', 'y']
            labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
            
            for i in range(6):
                line_pos, = self._traj_ax1.plot([], [], color=colors[i], label=labels[i])
                line_spd, = self._traj_ax2.plot([], [], color=colors[i], label=labels[i])
                self._traj_line_pos.append(line_pos)
                self._traj_line_spd.append(line_spd)
            
            self._traj_ax1.set_ylabel("target_pos")
            self._traj_ax2.set_ylabel("target_speed")
            self._traj_ax2.set_xlabel("t (s)")
            self._traj_ax1.grid(True)
            self._traj_ax2.grid(True)
            self._traj_ax1.legend(loc='upper right', fontsize=8)
            self._traj_ax2.legend(loc='upper right', fontsize=8)

            self._traj_init = True

        # Append latest samples
        t_now = t
        self._traj_t.append(t_now)
        
        target_pos_arr = np.asarray(target_pos).flatten()
        target_speed_arr = np.asarray(target_speed).flatten()
        
        for i in range(6):
            self._traj_pos[i].append(float(target_pos_arr[i]))
            self._traj_spd[i].append(float(target_speed_arr[i]))

        # Update plot
        x = np.fromiter(self._traj_t, dtype=float)
        
        for i in range(6):
            y_pos = np.fromiter(self._traj_pos[i], dtype=float)
            y_spd = np.fromiter(self._traj_spd[i], dtype=float)
            self._traj_line_pos[i].set_data(x, y_pos)
            self._traj_line_spd[i].set_data(x, y_spd)

        self._traj_ax1.relim()
        self._traj_ax1.autoscale_view()
        self._traj_ax2.relim()
        self._traj_ax2.autoscale_view()

        self._traj_fig.canvas.draw_idle()
        plt.pause(0.001)
    
    def _normalize(self, v):
        norm = np.linalg.norm(v)
        if norm < 1e-9:
            return np.zeros_like(v)
        return v / norm

    def _frame_from_zx(self, z_axis, x_direction):
        z_axis = self._normalize(z_axis)
        x_temp = x_direction - np.dot(x_direction, z_axis) * z_axis
        x_axis = self._normalize(x_temp)

        # Fallback if x_direction is nearly parallel to z_axis.
        if np.linalg.norm(x_axis) < 1e-9:
            fallback = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(fallback, z_axis)) > 0.9:
                fallback = np.array([0.0, 1.0, 0.0])
            x_axis = self._normalize(fallback - np.dot(fallback, z_axis) * z_axis)

        y_axis = self._normalize(np.cross(z_axis, x_axis))
        return x_axis, y_axis, z_axis

    def _angle_axis_to_rotation_matrix(self, angle_axis):
        angle_axis = np.asarray(angle_axis, dtype=float)
        angle = np.linalg.norm(angle_axis)

        if angle < 1e-9:
            return np.eye(3)

        axis = angle_axis / angle
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def _clear_tcp_orientation_artists(self):
        if not hasattr(self, "_tcp_ori_artists"):
            self._tcp_ori_artists = []
            return

        for artist in self._tcp_ori_artists:
            try:
                artist.remove()
            except Exception:
                pass
        self._tcp_ori_artists = []

    def _draw_tcp_orientation(self, tcp_pos, tcp_rot):
        if (not hasattr(self, "ax_3d")) or tcp_pos is None or tcp_rot is None:
            return

        pos = np.asarray(tcp_pos, dtype=float)
        rot = tcp_rot

        self._clear_tcp_orientation_artists()
        colors = ["r", "g", "b"]
        for i in range(3):
            axis = rot[:, i]
            q = self.ax_3d.quiver(
                pos[0], pos[1], pos[2],
                axis[0], axis[1], axis[2],
                length=self._tcp_axis_len,
                normalize=True,
                color=colors[i],
                linewidth=2,
                alpha=0.9,
            )
            self._tcp_ori_artists.append(q)
    
    def rotation_matrix_to_angle_axis(self, R):
        """Convert rotation matrix to angle-axis representation"""
        cos_angle = (np.trace(R) - 1) / 2
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        if angle < 1e-6:
            return np.zeros(3)
        
        # Calculate rotation axis from rotation matrix
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(angle))
        
        return angle * axis

    def angle_axis_to_rotation_matrix(self, angle_axis):
        """Convert angle-axis representation to rotation matrix"""
        angle = np.linalg.norm(angle_axis)
        
        if angle < 1e-6:
            return np.eye(3)
        
        axis = angle_axis / angle
        K = np.array([[0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]])
        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        return R
    
    def tcp_pose_estimate(self, v_desired, w_desired, t):
        dt = max(0.0, t - self.last_ctrl_t)
        if dt > 0.0:
            self.p_est = self.p_est + v_desired * dt
            dR = self.angle_axis_to_rotation_matrix(w_desired * dt)
            self.R_est = dR @ self.R_est
        self.last_ctrl_t = t
        self.realtime_path.append(self.p_est.copy())
        return np.hstack((self.p_est, self.rotation_matrix_to_angle_axis(self.R_est)))

    def init_3d_visualization(self, point_list, pointing_vectors, x_directions, axis_len=0.05):
        plt.ion()
        self.fig_3d = plt.figure(figsize=(9, 7))
        ax = self.fig_3d.add_subplot(111, projection="3d")
        self.ax_3d = ax
        self._tcp_axis_len = axis_len * 1.2
        self._tcp_ori_artists = []

        # Waypoint polyline
        ax.plot(point_list[:, 0], point_list[:, 1], point_list[:, 2], "k--", alpha=0.5, label="Waypoints")

        # Draw coordinate frame at each waypoint
        for i in range(point_list.shape[0]):
            p = point_list[i, :3]
            x_axis, y_axis, z_axis = self._frame_from_zx(pointing_vectors[i], x_directions[i])

            ax.quiver(p[0], p[1], p[2], x_axis[0], x_axis[1], x_axis[2], length=axis_len, normalize=True, color="r", linewidth=1)
            ax.quiver(p[0], p[1], p[2], y_axis[0], y_axis[1], y_axis[2], length=axis_len, normalize=True, color="g", linewidth=1)
            ax.quiver(p[0], p[1], p[2], z_axis[0], z_axis[1], z_axis[2], length=axis_len, normalize=True, color="b", linewidth=1)

        # Realtime trajectory line + current point marker
        self.traj_line, = ax.plot([], [], [], "m-", linewidth=2, label="TCP trajectory")
        self.current_pt, = ax.plot([], [], [], "mo", markersize=6)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("Waypoint Frames and Realtime 3D Trajectory")
        ax.legend(loc="upper right")

        # Make 3D scale roughly equal.
        xyz_min = np.min(point_list[:, :3], axis=0)
        xyz_max = np.max(point_list[:, :3], axis=0)
        center = (xyz_min + xyz_max) / 2.0
        span = np.max(xyz_max - xyz_min)
        span = max(span, 0.2)
        half = span / 2.0

        ax.set_xlim(center[0] - half, center[0] + half)
        ax.set_ylim(center[1] - half, center[1] + half)
        ax.set_zlim(center[2] - half, center[2] + half)

        self.realtime_path = [point_list[0, :3].copy()]
        self.p_est = point_list[0, :3].astype(float).copy()
        self.R_est = np.column_stack(self._frame_from_zx(pointing_vectors[0], x_directions[0]))
        self._draw_tcp_orientation(self.p_est, self.R_est)
    
        self.fig_3d.canvas.draw_idle()
        plt.pause(0.001)

    def update_3d_trajectory(self, tcp_pose=None):
        pts = np.asarray(self.realtime_path)
        self.traj_line.set_data(pts[:, 0], pts[:, 1])
        self.traj_line.set_3d_properties(pts[:, 2])
        self.current_pt.set_data([pts[-1, 0]], [pts[-1, 1]])
        self.current_pt.set_3d_properties([pts[-1, 2]])

        # Draw current TCP orientation axes (x/y/z) at the current TCP point.
        self._draw_tcp_orientation(tcp_pose[:3], self._angle_axis_to_rotation_matrix(tcp_pose[3:6]))

        self.fig_3d.canvas.draw_idle()
        plt.pause(0.001)

    def keep_plot_open(self):
        """Keep the trajectory plot open after execution."""
        if hasattr(self, "_traj_init") or hasattr(self, "fig_3d"):
            plt.ioff()
            plt.show()