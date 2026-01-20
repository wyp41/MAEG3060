from controller import RobotVelocityController

if __name__ == "__main__":
    controller = RobotVelocityController()
    controller.start()
    
    # Send joint velocity commands to robotic arm for 1 seconds
    # In a blocked way
    controller.speedJ([0.5, 0, 0, 0, 0, 0], 1)

    # Send joint velocity commands without blocking
    for i in range(500):
        controller.speedJ([0.5, 0, 0, 0, 0, 0])

        # Get and print current joint angles and TCP pose
        q = controller.get_current_q()
        print(f"Current joint angles: {q}")

        tcp_pose = controller.get_current_tcp_pose()
        print(f"Current TCP pose: {tcp_pose}")
    
    # Stop the robot
    controller.stop()

    # Close the connection
    controller.close()