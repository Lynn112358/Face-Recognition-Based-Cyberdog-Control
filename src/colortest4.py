import lcm
import sys
import time
import threading
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2

def find_camera_topic():
    """自动检测可用的相机话题"""
    try:
        # 创建临时节点来获取话题信息
        node = rclpy.create_node('temp_topic_detector')
        topics = node.get_topic_names_and_types()
        
        # 筛选图像话题
        image_topics = []
        for topic_name, topic_types in topics:
            if 'sensor_msgs/msg/Image' in topic_types:
                image_topics.append(topic_name)
        
        # 如果没有找到任何图像话题
        if not image_topics:
            print("未找到任何图像话题！")
            return None
        
        # 打印所有找到的图像话题
        print("找到的图像话题:")
        for topic in image_topics:
            print(f" - {topic}")
        
        # 优先选择包含"rgb"或"color"的话题
        for topic in image_topics:
            if 'rgb' in topic or 'color' in topic:
                print(f"选择话题: {topic}")
                return topic
        
        # 返回第一个图像话题（如果没有匹配的）
        print(f"没有找到包含'rgb'或'color'的图像话题，使用第一个图像话题: {image_topics[0]}")
        return image_topics[0]
    
    except Exception as e:
        print(f"查找相机话题时出错: {str(e)}")
        return None
    finally:
        # 确保销毁临时节点
        if 'node' in locals():
            node.destroy_node()

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
        
        # 初始化共享数据
        self.data_lock = Lock()  # 用于保护共享数据的锁
        self.result = None
        self.total_green = 0
        self.image_received_event = threading.Event()  # 图像接收事件
        self.image_received = False  # 标记是否接收到图像
        self.first_image_saved = False  # 标记是否已保存第一帧图像
        
        # 自动检测相机话题
        camera_topic = find_camera_topic()
        if camera_topic is None:
            self.get_logger().error("未找到相机话题！")
            return
        
        self.get_logger().info(f"使用相机话题: {camera_topic}")
        print(f"成功订阅相机话题: {camera_topic}")
        
        try:
            # 创建订阅 - 使用传感器数据的QoS配置
            self.subscription = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                qos_profile_sensor_data)
        except Exception as e:
            self.get_logger().error(f"创建订阅失败: {str(e)}")
            return
    
    def compare_red_green(self, cv_image):
        #优化后的颜色比较函数,使用HSV色彩空间和区域检测"""
    	try:
        # 转换到HSV色彩空间
        	hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 定义红色和绿色的HSV范围
        # 红色在HSV中有两个范围（因为它在色轮的两端）
        	lower_red1 = np.array([0, 120, 70])
        	upper_red1 = np.array([10, 255, 255])
        	lower_red2 = np.array([170, 120, 70])
        	upper_red2 = np.array([180, 255, 255])
        
        	lower_green = np.array([35, 50, 50])
        	upper_green = np.array([85, 255, 255])
        
        # 创建红色和绿色的掩膜
        	mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        	mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        	mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        	mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # 形态学操作（开运算）去除噪声
        	kernel = np.ones((5,5), np.uint8)
        	mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        	mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        
        # 计算红色和绿色区域的面积
        	red_area = cv2.countNonZero(mask_red)
        	green_area = cv2.countNonZero(mask_green)
         # 比较红色和绿色的总和
        	if red_area > green_area:
        	    return "Red"
        	elif green_area > red_area:
        	    return "Green"
        	else:
        	    return "Equal" 
            # 比较红色和绿色的总和    
        # 设置面积阈值，避免小噪声干扰
        	area_threshold = 500
        
             
    	except Exception as e:
        	self.get_logger().error(f"颜色比较出错: {str(e)}")
        	return "Error"

    def image_callback(self, msg):
        # 标记已接收到图像
        if not self.image_received:
            print("成功接收第一帧图像!")
            self.image_received = True
        self.image_received_event.set()
        
        # 将ROS 2的图像消息转换为OpenCV图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 保存第一帧图像用于调试
            if not self.first_image_saved:
                cv2.imwrite('first_frame.jpg', cv_image)
                print("已保存第一帧图像为 first_frame.jpg")
                self.first_image_saved = True
            
            # 调用颜色比较函数
            result = self.compare_red_green(cv_image)
            with self.data_lock:
                self.result = result
        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")
    
    def wait_for_first_image(self, timeout=10.0):
        """等待第一帧图像到达"""
        print(f"等待第一帧图像，超时: {timeout}秒...")
        
        # 如果已经接收到图像，立即返回
        if self.image_received:
            print("已经接收到图像")
            return True
        
        # 否则等待
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.image_received_event.is_set():
                print("成功接收第一帧图像")
                return True
            print(f"等待中... 已等待 {time.time()-start_time:.1f} 秒")
            # 处理一次ROS消息
            rclpy.spin_once(self, timeout_sec=0.5)
        
        print("等待第一帧图像超时！")
        return False
    
    def get_result(self):
        """获取颜色比较结果"""
        with self.data_lock:
            return self.result

def standup(ctrl, msg):  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count += 1  # Command will take effect when life_count update
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_10(12, 0)
    print("站立完成")

def color(ctrl, msg, color_comparison):  # 颜色识别
    # 等待第一帧图像
    if not color_comparison.wait_for_first_image(timeout=10.0):
        print("等待第一帧图像超时！无法进行颜色识别")
        return
    
    print("开始颜色识别...")
    
    msg.mode = 11 #locomotion模式
    msg.gait_id = 27 #子动作：慢走
    msg.vel_des = [0, -0.1, 0] 
    msg.duration = 2800#持续时间，单位ms
    msg.step_height = [0.05, 0.05]
    msg.life_count += 1
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_10(11, 27)

    vel = 0.2  # 左
    for i in range(5):
        result = color_comparison.get_result()
        print(f"颜色比较结果: {result}")
        if result == "Red":
            print("检测到红色，执行转向")
            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0, vel, 0]
            msg.duration = 3200
            msg.step_height = [0.05, 0.05]
            msg.life_count += 1
            ctrl.Send_cmd(msg)
            ctrl.Wait_finish_5(11, 27)
            vel = vel * -1

        msg.mode = 11
        msg.gait_id = 27
        msg.vel_des = [0.2, 0, 0]
        msg.duration = 4200
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_5(11, 27)

    msg.mode = 11
    msg.gait_id = 27
    msg.vel_des = [0.2, 0, 0]
    msg.duration = 4200
    msg.step_height = [0.05, 0.05]
    msg.life_count += 1
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_5(11, 27)

    msg.mode = 11
    msg.gait_id = 27
    msg.vel_des = [0, vel / 2, 0]
    msg.duration = 2800
    msg.step_height = [0.05, 0.05]
    msg.life_count += 1
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_5(11, 27)

    msg.mode = 7
    msg.gait_id = 0
    msg.life_count += 1
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_5(7, 0)
    print("颜色识别结束")

def spin_executor(executor, node):
    try:
        print("启动执行器...")
        executor.spin()
    except Exception as e:
        print(f"执行器运行时出错: {str(e)}")
    finally:
        # 确保节点在程序退出时被正确销毁
        node.destroy_node()
        print("颜色比较节点已销毁")

# 主程序
if __name__ == '__main__':
    executor = None
    spin_thread = None
    ctrl = None
    
    try:
        print("初始化ROS 2...")
        # 初始化ROS 2
        rclpy.init()
        
        # 创建颜色比较节点
        print("创建颜色比较节点...")
        color_comparison = ColorComparison()
        
        # 检查节点是否有效创建
        if not hasattr(color_comparison, 'subscription'):
            print("相机订阅创建失败，无法继续执行!")
            # 清理资源
            color_comparison.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        
        # 创建执行器
        print("创建执行器...")
        executor = MultiThreadedExecutor()
        executor.add_node(color_comparison)
        
        # 启动执行器线程
        print("启动执行器线程...")
        spin_thread = Thread(target=spin_executor, args=(executor, color_comparison))
        spin_thread.daemon = True  # 设置为守护线程
        spin_thread.start()
        
        # 初始化LCM控制
        print("初始化LCM控制...")
        ctrl = Robot_Ctrl()
        ctrl.run()
        msg = robot_control_cmd_lcmt()
        
        # 等待ROS节点初始化
        print("等待节点初始化...")
        time.sleep(3)
        
        # 执行主要功能
        print("执行站立动作...")
        standup(ctrl, msg)  # 站立
        print("起立完成")
        
        # 执行颜色识别
        color(ctrl, msg, color_comparison)
        
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        print("清理资源...")
        try:
            # 停止LCM控制
            if ctrl:
                ctrl.quit()
            
            # 停止执行器
            if executor:
                executor.shutdown()
            
            # 停止执行器线程
            if spin_thread and spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            
            # 关闭ROS 2
            rclpy.shutdown()
        except Exception as e:
            print(f"资源清理时出错: {str(e)}")
        
        print("程序退出")
        sys.exit()
