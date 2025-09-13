# colortest3_face_recognition.py
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
import pickle
import os
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC

# 人脸识别模型类
class FaceRecognizer:
    def __init__(self, model_path="models/face_recognition_model.pkl"):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # 加载训练好的模型
        if os.path.exists(model_path):
            with open(model_path, 'rb') as f:
                self.model, self.label_encoder = pickle.load(f)
            print("人脸识别模型加载成功")
        else:
            print("未找到训练好的模型，请先运行训练脚本")
            self.model = None
            self.label_encoder = None
    
    def preprocess_face(self, face_image):
        """预处理人脸图像"""
        # 转换为灰度图
        if len(face_image.shape) == 3:
            gray = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = face_image
        
        # 直方图均衡化
        gray = cv2.equalizeHist(gray)
        
        # 调整大小
        gray = cv2.resize(gray, (100, 100))
        
        # 归一化
        gray = gray / 255.0
        
        return gray.flatten()
    
    def recognize_face(self, frame):
        """识别图像中的人脸"""
        if self.model is None or self.label_encoder is None:
            return None, None, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 检测人脸
        faces = self.face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.1, 
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        recognized_faces = []
        
        for (x, y, w, h) in faces:
            # 提取人脸区域
            face_roi = frame[y:y+h, x:x+w]
            
            # 预处理
            processed_face = self.preprocess_face(face_roi)
            
            # 预测
            prediction = self.model.predict_proba([processed_face])
            max_prob = np.max(prediction)
            person_id = np.argmax(prediction)
            
            # 只接受置信度高于阈值的识别结果
            if max_prob > 0.7:  # 阈值可根据实际情况调整
                person_name = self.label_encoder.inverse_transform([person_id])[0]
                recognized_faces.append((person_name, max_prob, (x, y, w, h)))
        
        return recognized_faces

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


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.bridge = CvBridge()
        
        # 初始化人脸识别器
        self.face_recognizer = FaceRecognizer()
        
        # 初始化共享数据
        self.data_lock = Lock()
        self.recognized_persons = []  # 存储识别到的人脸信息
        self.image_received_event = threading.Event()
        self.image_received = False
        self.first_image_saved = False
        
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
            
            # 识别人脸
            recognized_faces = self.face_recognizer.recognize_face(cv_image)
            
            with self.data_lock:
                self.recognized_persons = recognized_faces if recognized_faces else []
                
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
    
    def get_recognized_persons(self):
        """获取识别到的人脸信息"""
        with self.data_lock:
            return self.recognized_persons

def standup(ctrl, msg):  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count += 1  # Command will take effect when life_count update
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish_10(12, 0)
    print("站立完成")

def person_specific_action(ctrl, msg, person_name):
    """根据识别到的人员执行特定动作"""
    print(f"识别到 {person_name}，执行特定动作")
    
    # 根据不同人员执行不同动作
    if person_name == "陈佳":
        # 执行person1的特定动作 - 握手
        msg.mode = 62
        msg.gait_id = 1
        msg.vel_des = [0, 0, 0] 
        msg.duration = 8700
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_5(11, 27)
        
    elif person_name == "李晨曦":
        # 执行person2的特定动作 - 太空步
        msg.mode = 62
        msg.gait_id = 12
        msg.vel_des = [0.2, 0, 0]  
        msg.duration = 9100
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_5(11, 27)

    elif person_name == "唐佳乐":
        # 执行person3的特定动作 - 扭屁股
        msg.mode = 62
        msg.gait_id = 4
        msg.vel_des = [0, 0, 0]  
        msg.duration = 4000
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_5(11, 27)
            
    elif person_name == "刘翔宇":
        # 执行person4的特定动作 -跳起
        msg.mode = 16
        msg.gait_id = 6
        msg.vel_des = [0, 0, 0]  
        msg.duration = 780
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_5(11, 27)
       
        msg.mode = 12
        msg.gait_id = 0
        msg.vel_des = [0, 0, 0] 
        msg.duration = 0
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_3(11, 27)
    else:
        # 默认动作 - 点头
        msg.mode = 11
        msg.gait_id = 27
        msg.vel_des = [0, 0, 0.1]  # 身体上下运动
        msg.duration = 1000
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_3(11, 27)
        
        msg.mode = 11
        msg.gait_id = 27
        msg.vel_des = [0, 0, -0.1]  # 恢复
        msg.duration = 1000
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish_3(11, 27)

def face_recognition_demo(ctrl, msg, face_node):
    """人脸识别演示"""
    # 等待第一帧图像
    if not face_node.wait_for_first_image(timeout=10.0):
        print("等待第一帧图像超时！无法进行人脸识别")
        return
    
    print("开始人脸识别演示...")
    
    # 执行一段时间的人脸识别
    start_time = time.time()
    recognition_duration = 120  # 演示30秒
    
    last_recognized_persons = []
    
    while time.time() - start_time < recognition_duration:
        # 获取识别到的人脸
        recognized_persons = face_node.get_recognized_persons()
        
        if recognized_persons and recognized_persons != last_recognized_persons:
            for person_name, confidence, _ in recognized_persons:
                print(f"识别到: {person_name}, 置信度: {confidence:.2f}")
                
                # 执行特定人员的动作
                person_specific_action(ctrl, msg, person_name)
            
            last_recognized_persons = recognized_persons
        
        time.sleep(1)  # 每秒检查一次
    
    print("人脸识别演示结束")

def spin_executor(executor, node):
    try:
        print("启动执行器...")
        executor.spin()
    except Exception as e:
        print(f"执行器运行时出错: {str(e)}")
    finally:
        # 确保节点在程序退出时被正确销毁
        node.destroy_node()
        print("人脸识别节点已销毁")

# 主程序
if __name__ == '__main__':
    executor = None
    spin_thread = None
    ctrl = None
    
    try:
        print("初始化ROS 2...")
        # 初始化ROS 2
        rclpy.init()
        
        # 创建人脸识别节点
        print("创建人脸识别节点...")
        face_node = FaceRecognitionNode()
        
        # 检查节点是否有效创建
        if not hasattr(face_node, 'subscription'):
            print("相机订阅创建失败，无法继续执行!")
            # 清理资源
            face_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        
        # 创建执行器
        print("创建执行器...")
        executor = MultiThreadedExecutor()
        executor.add_node(face_node)
        
        # 启动执行器线程
        print("启动执行器线程...")
        spin_thread = Thread(target=spin_executor, args=(executor, face_node))
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
        
        # 执行人脸识别演示
        face_recognition_demo(ctrl, msg, face_node)
        
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