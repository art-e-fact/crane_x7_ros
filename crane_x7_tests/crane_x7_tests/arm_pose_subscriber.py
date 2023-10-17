import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("arm_pose_state_subscriber")
        self.subscription = self.create_subscription(
            PoseArray, "/world/default/dynamic_pose/info", self.arm_pose_callback, 10
        )

        self.subscription

    def arm_pose_callback(self, msg):
        # rounded_joint_values = []
        # decimal_place = 2
        # for i in range(len(msg.position)):
        #     rounded_joint = round(msg.position[i], decimal_place)

        #     if rounded_joint == -0.0:
        #         rounded_joint = 0.0

        #     rounded_joint_values.append(rounded_joint)

        self.get_logger().info('arm_pose: "%s" \n' % msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_subscriber = JointStateSubscriber()

    rclpy.spin(joint_state_subscriber)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
