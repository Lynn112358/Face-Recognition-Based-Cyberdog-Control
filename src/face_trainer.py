# face_trainer.py
import cv2
import os
import numpy as np
import pickle
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

class FaceTrainer:
    def __init__(self, data_dir="face_data", model_dir="models"):
        self.data_dir = data_dir
        self.model_dir = model_dir
        
        if not os.path.exists(model_dir):
            os.makedirs(model_dir)
    
    def load_training_data(self):
        """加载训练数据"""
        faces = []
        labels = []
        
        # 遍历每个人物的文件夹
        for person_name in os.listdir(self.data_dir):
            person_dir = os.path.join(self.data_dir, person_name)
            
            if not os.path.isdir(person_dir):
                continue
            
            # 遍历每个人物的所有图像
            for image_name in os.listdir(person_dir):
                image_path = os.path.join(person_dir, image_name)
                
                # 读取图像
                image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
                
                if image is not None:
                    # 直方图均衡化增强对比度
                    image = cv2.equalizeHist(image)
                    
                    # 归一化
                    image = image / 255.0
                    
                    # 转换为1D数组
                    faces.append(image.flatten())
                    labels.append(person_name)
        
        return np.array(faces), np.array(labels)
    
    def train_model(self):
        """训练人脸识别模型"""
        print("加载训练数据...")
        faces, labels = self.load_training_data()
        
        if len(faces) == 0:
            print("没有找到训练数据")
            return False
        
        print(f"加载了 {len(faces)} 张训练图像")
        
        # 编码标签
        le = LabelEncoder()
        labels_encoded = le.fit_transform(labels)
        
        # 划分训练集和测试集
        X_train, X_test, y_train, y_test = train_test_split(
            faces, labels_encoded, test_size=0.2, random_state=42
        )
        
        print("训练SVM分类器...")
        # 使用SVM分类器
        model = SVC(kernel='linear', probability=True, random_state=42)
        model.fit(X_train, y_train)
        
        # 评估模型
        y_pred = model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        print(f"模型准确率: {accuracy:.2f}")
        
        # 保存模型
        model_path = os.path.join(self.model_dir, "face_recognition_model.pkl")
        with open(model_path, 'wb') as f:
            pickle.dump((model, le), f)
        
        print(f"模型已保存到 {model_path}")
        
        # 绘制准确率图表
        self.plot_accuracy(accuracy, len(np.unique(labels_encoded)))
        
        return True
    
    def plot_accuracy(self, accuracy, num_classes):
        """绘制准确率图表"""
        plt.figure(figsize=(8, 6))
        plt.bar(['Accuracy'], [accuracy], color='blue')
        plt.ylim(0, 1)
        plt.title(f'Face Recognition Model Accuracy (Trained on {num_classes} classes)')
        plt.ylabel('Accuracy')
        plt.text(0, accuracy + 0.02, f'{accuracy:.2f}', ha='center')
        plt.savefig(os.path.join(self.model_dir, 'accuracy_plot.png'))
        plt.close()

if __name__ == "__main__":
    trainer = FaceTrainer()
    trainer.train_model()