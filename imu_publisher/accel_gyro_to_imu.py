import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class IMUCombiner(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.imu_publisher = self.create_publisher(Imu, '/imu', qos_profile)
        self.accel_subscriber = self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.accel_callback,
            qos_profile)
        self.gyro_subscriber = self.create_subscription(
            Imu,
            '/camera/camera/gyro/sample',
            self.gyro_callback,
            qos_profile)
        self.latest_accel = None
        self.latest_gyro = None

    def accel_callback(self, msg):
        self.latest_accel = msg

    def gyro_callback(self, msg):
        self.latest_gyro = msg
        if self.latest_accel is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_frame'
            imu_msg.linear_acceleration = self.latest_accel.linear_acceleration
            imu_msg.angular_velocity = self.latest_gyro.angular_velocity
            self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_combiner = IMUCombiner()
    rclpy.spin(imu_combiner)
    imu_combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

