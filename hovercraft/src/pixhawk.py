#/mavros/imu/data -> quaternions imu
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

sefl.yaw = rospy.Subscriber("/mavros/imu/data", Imu, self.callback)

def convert_graus(self, valor):
    orientation = valor.orientation
    quaternion = [orientation.x,orientation.y,orientation.z,orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    yaw *= 360/(2*math.pi)
    return yaw