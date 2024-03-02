import math
import rclpy
from sympy import sin,cos
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MotionPublisher(Node):

    def __init__(self):
        super().__init__('motion_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
############################## variaveis de junta ##############################
        self.minimum_limit_shoulder_pitch = -2.618
        self.minimum_limit_shoulder_roll = -3.14
        self.minimum_limit_arm_yaw = -1.57
        self.minimum_limit_elbow_pitch = -2.182
        self.minimum_limit_forearm_yaw = -1.745
        self.minimum_limit_wrist_pitch = -0.785
        self.minimum_limit_wrist_roll = -0.785
        self.minimum_limit_gripper = -1.2

        self.maximum_limit_shoulder_pitch = 1.57
        self.maximum_limit_shoulder_roll = 0.174
        self.maximum_limit_arm_yaw = 1.57
        self.maximum_limit_elbow_pitch = 0.
        self.maximum_limit_forearm_yaw = 1.745
        self.maximum_limit_wrist_pitch = 0.785
        self.maximum_limit_wrist_roll = 0.785
        self.maximum_limit_gripper = 0.350

        self.shoulder_pitch = 0.
        self.shoulder_roll = 0.
        self.arm_yaw = 0.
        self.elbow_pitch = 0.
        self.forearm_yaw = 0.
        self.wrist_pitch = 0.
        self.wrist_roll = 0.
        self.gripper = 0.
        self.tip = 0.
############################### variaveis de posição #############################
        self.x0 = 0.
        self.y0 = 0.
        self.z0 = 0.
        self.x1 = 0.
        self.y1 = 0.
        self.z1 = 0.
        self.desired_point = [190.0, 637., 0]
        self.arm_length = 637.5

        self.i = 0
        self.flag = 0

        self.joint_state = JointState()

    def timer_callback(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw','r_elbow_pitch','r_forearm_yaw','r_wrist_pitch','r_wrist_roll']
        self.joint_state.position = [self.shoulder_pitch, self.shoulder_roll, self.arm_yaw, self.elbow_pitch, self.forearm_yaw, self.wrist_pitch, self.wrist_roll]
        self.publisher_.publish(self.joint_state)
        
        self.efector_distance, self.x0, self.y0, self.z0 = self.efector_position(self.shoulder_pitch, self.shoulder_roll, self.arm_yaw, self.elbow_pitch, self.forearm_yaw, self.wrist_pitch, self.wrist_roll)
        self.point_distance = self.desired_position(self.desired_point[0], self.desired_point[1], self.desired_point[2])
        
        if self.efector_distance > self.point_distance and self.point_distance > 300.1:
            if self.i > 500:
                if self.elbow_pitch > self.minimum_limit_elbow_pitch:
                    self.elbow_pitch = self.elbow_pitch - 0.01
                    self.flag = 1
                    #self.get_logger().info('X: "%.6s" Y: "%.6s" Z: "%.6s" DX: "%.6s" DY: "%.6s" DZ: "%.6s"' % (self.x0, self.y0, self.z0,self.desired_point[0],self.desired_point[1],self.desired_point[2]))
        
        elif self.efector_distance < self.point_distance or self.point_distance < 300.1:
            if self.flag == 1:
                self.get_logger().info('X: "%.6s" Y: "%.6s" Z: "%.6s" DX: "%.6s" DY: "%.6s" DZ: "%.6s"' % (self.x0, self.y0, self.z0,self.desired_point[0],self.desired_point[1],self.desired_point[2]))
            else:
                self.get_logger().info('Posição inalcançavel')

        if self.desired_point[2] > self.z0:
            if self.i > 500:
                if self.shoulder_pitch > self.minimum_limit_shoulder_pitch:
                    self.shoulder_pitch = self.shoulder_pitch - 0.001
                    #self.get_logger().info('X: "%.6s" Y: "%.6s" Z: "%.6s" DX: "%.6s" DY: "%.6s" DZ: "%.6s"' % (self.x0, self.y0, self.z0,self.desired_point[0],self.desired_point[1],self.desired_point[2]))

        if self.desired_point[2] < self.z0:
            if self.i > 500:
                if self.shoulder_pitch < self.maximum_limit_shoulder_pitch:
                    self.shoulder_pitch = self.shoulder_pitch + 0.001
                    #self.get_logger().info('X: "%.6s" Y: "%.6s" Z: "%.6s" DX: "%.6s" DY: "%.6s" DZ: "%.6s"' % (self.x0, self.y0, self.z0,self.desired_point[0],self.desired_point[1],self.desired_point[2]))

        if self.i > 500:
            self.i = 500
        else:
            self.i += 1

    def efector_position(self, q1, q2, q3, q4, q5, q6, q7):
        x1 = 190.
        y1 = 0.
        z1 = 0.
        x0 = -100.0*sin(q1)*sin(q2)*cos(q3)*cos(q4) - 150.0*sin(q1)*sin(q2)*cos(q3)*cos(q4 + q5) - 107.5*sin(q1)*sin(q2)*cos(q3)*cos(q4 + q5 + q6) - 243.0*sin(q1)*sin(q2)*cos(q3) - 37.0*sin(q1)*sin(q2) - 100.0*sin(q1)*sin(q4)*cos(q2) - 150.0*sin(q1)*sin(q4 + q5)*cos(q2) - 107.5*sin(q1)*sin(q4 + q5 + q6)*cos(q2) - 100.0*sin(q3)*cos(q1)*cos(q4) - 150.0*sin(q3)*cos(q1)*cos(q4 + q5) - 107.5*sin(q3)*cos(q1)*cos(q4 + q5 + q6) - 243.0*sin(q3)*cos(q1) + 47.0*cos(q1) + 143.0
        y0 = -100.0*sin(q1)*sin(q3)*cos(q4) - 150.0*sin(q1)*sin(q3)*cos(q4 + q5) - 107.5*sin(q1)*sin(q3)*cos(q4 + q5 + q6) - 243.0*sin(q1)*sin(q3) + 47.0*sin(q1) + 100.0*sin(q2)*cos(q1)*cos(q3)*cos(q4) + 150.0*sin(q2)*cos(q1)*cos(q3)*cos(q4 + q5) + 107.5*sin(q2)*cos(q1)*cos(q3)*cos(q4 + q5 + q6) + 243.0*sin(q2)*cos(q1)*cos(q3) + 37.0*sin(q2)*cos(q1) + 100.0*sin(q4)*cos(q1)*cos(q2) + 150.0*sin(q4 + q5)*cos(q1)*cos(q2) + 107.5*sin(q4 + q5 + q6)*cos(q1)*cos(q2)
        z0 = 100.0*sin(q2)*sin(q4) + 150.0*sin(q2)*sin(q4 + q5) + 107.5*sin(q2)*sin(q4 + q5 + q6) - 100.0*cos(q2)*cos(q3)*cos(q4) - 150.0*cos(q2)*cos(q3)*cos(q4 + q5) - 107.5*cos(q2)*cos(q3)*cos(q4 + q5 + q6) - 243.0*cos(q2)*cos(q3) - 37.0*cos(q2)
        self.get_logger().info('%s'%q1)
        resultant_distance = math.sqrt(((x1 - x0)*(x1 - x0)) + ((y1 - y0)*(y1 - y0)) + ((z1 - z0)*(z1 - z0)))

        return resultant_distance, x0, y0, z0
        
    def desired_position(self,x1, y1, z1):
        x0 = 190.
        y0 = 0.
        z0 = 0.
        resultant_distance = math.sqrt(((x1 - x0)*(x1 - x0)) + ((y1 - y0)*(y1 - y0)) + ((z1 - z0)*(z1 - z0)))

        return resultant_distance

def main(args=None):
    rclpy.init(args=args)

    motion_publisher = MotionPublisher()

    rclpy.spin(motion_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motion_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()