import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('topic_desired_trajectory_publisher_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    def timer_callback(self):
        # Get the current end-effector pose
        end_effector_pose = self.get_end_effector_pose()

        # Calculate the inverse kinematics to get joint angles
        joint_angles = self.calculate_inverse_kinematics(end_effector_pose)

        # Publish joint trajectory
        self.publish_joint_trajectory(joint_angles)

    def get_end_effector_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_0', 'link6', rclpy.time.Time().to_msg(), rclpy.time.Duration(seconds=1))
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            return pose_stamped
        except Exception as e:
            self.get_logger().error(f"Error getting transform: {str(e)}")
            return None

    def calculate_inverse_kinematics(self, end_effector_pose):
        # Implement your inverse kinematics calculations here
        # For a simple example, you can print the end-effector position
        if end_effector_pose:
            self.get_logger().info(f"End-effector Pose (x, y, z): ({end_effector_pose.pose.position.x}, {end_effector_pose.pose.position.y}, {end_effector_pose.pose.position.z})")
            # Perform inverse kinematics calculations to get joint angles
            # ...

    def publish_joint_trajectory(self, joint_angles):
        if joint_angles:
            # Create a JointTrajectory message and publish it
            point_msg = JointTrajectoryPoint()
            point_msg.positions = joint_angles
            point_msg.time_from_start = Duration(sec=2)

            joints = ['joint1', 'joint2', 'joint3']
            my_trajectory_msg = JointTrajectory()
            my_trajectory_msg.joint_names = joints
            my_trajectory_msg.points.append(point_msg)

            self.trajectory_publisher.publish(my_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
