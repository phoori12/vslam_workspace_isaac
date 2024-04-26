import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster


class SLAM_Master(Node):

    def __init__(self):
        super().__init__('slam_master')

        # QOS Policies for TF Broadcaster
        # qos_profile = QoSProfile(
        #     durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     depth=1
        # )

        # Init Publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Init Subscriber
        self.vo_pose_subscriber = self.create_subscription(
            PoseStamped,
            'visual_slam/tracking/vo_pose',
            self.vo_pose_callback,
            10)
        self.vo_pose_subscriber  # prevent unused variable warning

        # TF STUFF
        self.prev_t_0 = TransformStamped()
        self.prev_t_1 = TransformStamped()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.target_frame_0 = self.declare_parameter(
          'target_frame_0', 'tag36h11:0').get_parameter_value().string_value
        
        self.target_frame_1 = self.declare_parameter(
          'target_frame_1', 'tag36h11:1').get_parameter_value().string_value
        
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'tag36h11:0_transient').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        

    def vo_pose_callback(self, msg):
        msgnew = msg
        #self.get_logger().info('I heard: "%s"' % msg.pose)

    
    def timer_callback(self):
        # Store frame names in variables that will be used to
        # compute transformations
        to_frame_rel = 'map'

        try:
            t_0 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                self.target_frame_0,
                rclpy.time.Time())
            self.prev_t_0 = t_0
        except TransformException as ex:
            m = "1"
            
        
        try:
            t_1 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                self.target_frame_1,
                rclpy.time.Time())
            self.prev_t_1 = t_1
        except TransformException as ex:
            m = "1"
            #self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        
        self.prev_t_1.header.stamp = self.get_clock().now().to_msg()
        self.prev_t_1.header.frame_id = 'map'
        self.prev_t_1.child_frame_id = 'tag36h11:1_transient'
        self.prev_t_0.header.stamp = self.get_clock().now().to_msg()
        self.prev_t_0.header.frame_id = 'map'
        self.prev_t_0.child_frame_id = 'tag36h11:0_transient'
        self.tf_broadcaster.sendTransform([self.prev_t_0,self.prev_t_1])
        

        
        #self.get_logger().info(t.transform.translation.x)


        #self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    slam_master = SLAM_Master()

    rclpy.spin(slam_master)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    slam_master.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()