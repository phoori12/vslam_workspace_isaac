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
        # self.prev_t_0 = TransformStamped()
        # self.prev_t_1 = TransformStamped()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_t = []
        self.target_frames = [] 
        self.publish_markers = []
        for i in range(8):
            target_name = "target_frame_" + str(i)
            tag_name = 'tag36h11:' + str(i)
            self.target_frames.append(self.declare_parameter(target_name, tag_name).get_parameter_value().string_value)
            self.prev_t.append(TransformStamped())
            self.publish_markers.append(False)

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

        frames_publish = []
        for i in range(8):
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    self.target_frames[i],
                    rclpy.time.Time())
                self.prev_t[i] = t
                self.publish_markers[i] = False
            except TransformException as ex:
                self.publish_markers[i] = True
                
            self.prev_t[i].header.stamp = self.get_clock().now().to_msg()
            self.prev_t[i].header.frame_id = 'map'
            self.prev_t[i].child_frame_id = 'tag36h11:' + str(i) + "_static"
            if self.publish_markers[i] == True:
                frames_publish.append(self.prev_t[i])

        self.tf_broadcaster.sendTransform(frames_publish)



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