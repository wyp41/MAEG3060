import time
from controller import RobotVelocityController
import numpy as np

if __name__ == "__main__":
    controller = RobotVelocityController("127.0.0.1")
    controller.start()

    # Send joint velocity commands without blocking
    # Command is sent at 125Hz
    for i in range(500):
        controller.speedJ(np.array([0.1, 0, 0, 0, 0, 0]))

        # Get and print current joint angles and TCP pose
        q = controller.get_current_q()
        print(f"Current joint angles: {q}")

        tcp_pose = controller.get_current_tcp_pose()
        print(f"Current TCP pose: {tcp_pose}")
    
    # Stop the robot
    controller.stop()

    # Close the connection
    controller.close()