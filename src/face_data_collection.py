
# face_data_collection.py
import cv2
import os
import numpy as np
from datetime import datetime

class FaceDataCollector:
    def __init__(self, data_dir="face_data"):
        self.data_dir = data_dir
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # 创建数据目录
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
    
    def collect_data(self, person_name, sample_count=50):
        """收集指定人员的人脸数据"""
        person_dir = os.path.join(self.data_dir, person_name)
        if not os.path.exists(person_dir):
            os.makedirs(person_dir)
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("无法打开摄像头")
            return False
        
        count = 0
        print(f"开始收集 {person_name} 的人脸数据，请面对摄像头...")
        
        while count < sample_count:
            ret, frame = cap.read()
            if not ret:
                print("无法获取帧")
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.1, 
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            for (x, y, w, h) in faces:
                # 提取人脸区域
                face_roi = gray[y:y+h, x:x+w]
                
                # 调整大小
                face_roi = cv2.resize(face_roi, (100, 100))
                
                # 保存图像
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{person_name}_{timestamp}.jpg"
                filepath = os.path.join(person_dir, filename)
                cv2.imwrite(filepath, face_roi)
                
                count += 1
                print(f"已收集 {count}/{sample_count} 张图像")
                
                # 绘制矩形框
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # 显示画面
            cv2.imshow('数据收集 - 按ESC退出', frame)
            
            # 按ESC键退出
            if cv2.waitKey(1) == 27:
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print(f"{person_name} 的数据收集完成")
        return True

if __name__ == "__main__":
    collector = FaceDataCollector()
    
    # 收集多个人员的数据
    persons = ["陈佳",  "李晨曦","唐佳乐","刘翔宇"] 
    
    for person in persons:
        input(f"准备收集 {person} 的数据，按回车键继续...")
        collector.collect_data(person, sample_count=50)
    
    print("所有人员数据收集完成")