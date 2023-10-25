import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("arm_pose_state_subscriber")
        self.subscription = self.create_subscription(
            PoseArray,
            "/world/default/dynamic_pose/info",
            self.gripper_pose_callback,
            10,
        )

        self.subscription
        self.gripper_x_pose_list = []
        self.gripper_y_pose_list = []

    def gripper_pose_callback(self, msg):
        """
        Subscriber callback for getting the pose of the Crane_X7 gripper link x and y positions
        """
        gripper_index_value = -3

        gripper_pose = msg.poses[gripper_index_value]
        gripper_x_pose = gripper_pose.position.x
        gripper_y_pose = gripper_pose.position.y

        self.gripper_x_pose_list.append(gripper_x_pose)
        self.gripper_y_pose_list.append(gripper_y_pose)

        # print(gripper_x_pose)
        # self.get_logger().info('arm_pose: "%s" \n' % msg.poses[gripper_index_value])


def main(args=None):
    rclpy.init(args=args)

    joint_state_subscriber = JointStateSubscriber()

    rclpy.spin(joint_state_subscriber)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
