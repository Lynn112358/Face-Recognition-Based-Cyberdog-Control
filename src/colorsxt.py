import lcm
import sys
import time
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.signal import savgol_filter
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2


class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if (self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish_15(self, mode, gait_id):
        count = 0
        while self.runing and count < 3000:  # 10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_10(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000:  # 10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_5(self, mode, gait_id):
        count = 0
        while self.runing and count < 1000:  # 5s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_3(self, mode, gait_id):
        count = 0
        while self.runing and count < 600:  # 3s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20:  # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()


class ColorComparison(Node):
    def __init__(self):
        super().__init__('color_comparison')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            qos_profile)
        self.subscription  # 防止未使用变量的警告
        self.result = None
        self.data_lock = Lock()  # 用于保护共享数据的锁

    def image_callback(self, msg):
        # 将ROS 2的图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 调用颜色比较函数
        result = self.compare_red_green(cv_image)
        with self.data_lock:
            self.result = result

    def compare_red_green(self, cv_image):
        # 将BGR图像转换为RGB图像
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # 分离红色、绿色通道
        red_channel = rgb_image[:, :, 0]
        green_channel = rgb_image[:, :, 1]

        # 计算红色和绿色通道的总和
        total_red = np.sum(red_channel)
        total_green = np.sum(green_channel)

        # 比较红色和绿色的总和
        if total_red > total_green:
            return "Red",total_red  # 红多
        elif total_green > total_red:
            return "Green",total_green  # 绿多
        else:
            return None  # 一样多

    def get_result(self):
        with self.data_lock:
            return self.result

def standup():  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count += 1  # Command will take effect when life_count update
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_10(12, 0)
def color():  # 颜色识别
    msg.mode = 11 #locomotion模式
    msg.gait_id = 27 #子动作：慢走
    msg.vel_des = [0, -0.1, 0] 
    msg.duration = 2800#持续时间，单位ms
    msg.step_height = [0.05, 0.05]
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_10(11, 27)

    vel = 0.2  # 左
    
    for i in range(5):
        result_tuple = color_comparison.get_result()
        print(f"Color comparison result: {result_tuple}")
        
        if result_tuple and isinstance(result_tuple, tuple) and len(result_tuple) > 0:
            result = result_tuple[0]
            
            if result == "Red":
                print("检测到红色，执行红色动作")
                msg.mode = 11
                msg.gait_id = 27
                msg.vel_des = [0, vel, 0]
                msg.duration = 3200
                msg.step_height = [0.05, 0.05]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish_5(11, 27)
                vel = vel * -1
                return  # 直接返回，不执行后面的动作
          
            elif result == "Green":
                print("检测到绿色，执行绿色动作")
                msg.mode = 62
                msg.gait_id = 11
                msg.vel_des = [0.2, 0, 0]
                msg.duration = 5200
                msg.step_height = [0.03, 0.03]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish_5(62, 11)
                #time.sleep(5)
                return  # 直接返回，不执行后面的动作
        
        time.sleep(0.1)
    
    # 如果循环结束都没有检测到颜色，执行默认动作
    print("未检测到特定颜色，执行默认动作")
    msg.mode = 11
    msg.gait_id = 2
    msg.vel_des = [0, vel / 2, 0]
    msg.duration = 2800
    msg.step_height = [0.05, 0.05]
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_5(11, 27)

    # 最终停止动作
    msg.mode = 7
    msg.gait_id = 0
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_5(7, 0)




def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        color_comparison.destroy_node()

        rclpy.shutdown()


Ctrl = Robot_Ctrl()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()
color_comparison = ColorComparison()


executor = MultiThreadedExecutor()

# 添加节点到执行器
executor.add_node(color_comparison)


spin_thread = Thread(target=spin_executor)
spin_thread.start()


def main():
    try:
        standup()  # 站立
        print("起立")
        color()  # 颜色识别
        print("颜色识别结束")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        Ctrl.quit()
        sys.exit()


if __name__ == '__main__':
    main()
