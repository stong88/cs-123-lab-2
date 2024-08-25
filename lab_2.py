import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            'leg_front_r_end_effector_position',
            10)
        timer_period = 0.5  # seconds (2 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_positions = None

    def listener_callback(self, msg):
        # Extract the positions of the joints related to leg_front_r
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = [msg.position[msg.name.index(joint)] for joint in joints_of_interest]

    def forward_kinematics(self, theta1, theta2, theta3):
        def rotation_x(angle):
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

        def rotation_y(angle):
            return 
            ## TODO: Implement the rotation matrix about the y-axis
            # return np.array([
            # ])
        
        def rotation_z(angle):
            return
            ## TODO: Implement the rotation matrix about the z-axis
            # return np.array([
            # ])

        def translation(x, y, z):
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
        T_1_2 = rotation_y(-1.57080) @ rotation_z(None)

        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
        T_2_3 = translation(None) @ rotation_y(None) @ rotation_z(None)

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.01800)

        # Compute the final transformation
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # Extract the end-effector position
        end_effector_position = T_0_ee[:3, 3]

        return end_effector_position


    def timer_callback(self):
        if self.joint_positions is not None:
            # Joint angles
            theta1 = self.joint_positions[0]
            theta2 = self.joint_positions[1]
            theta3 = self.joint_positions[2]

            end_effector_position = self.forward_kinematics(theta1, theta2, theta3)

            position = Float64MultiArray()
            position.data = end_effector_position

            self.position_publisher.publish(position)
            # self.get_logger().info(f'theta1 = {theta1:.1f}, theta2 = {theta2:.1f}, theta3 = {theta3:.1f}')
            self.get_logger().info(f'End-Effector Position: x={end_effector_position[0]:.2f}, y={end_effector_position[1]:.2f}, z={end_effector_position[2]:.2f}')

def main(args=None):
    rclpy.init(args=args)

    forward_kinematics = ForwardKinematics()

    rclpy.spin(forward_kinematics)

    forward_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
