import glob
import os
import serial
import serial.tools.list_ports
import time
import cv2
import numpy as np
import psutil
from datetime import datetime
import multiprocessing
from insightface.app import FaceAnalysis
from tqdm import tqdm
import sys
sys.setrecursionlimit(sys.getrecursionlimit() * 5)

# ---------------- Utilities ----------------
def is_locked_by_logonui():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == "LogonUI.exe": return True
    return False

def save_frame(frame):
    folder = "recognized_faces"
    os.makedirs(folder, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(folder, f"{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"📸 已保存识别图像到: {filename}")

def calculate_similarity(embedding1, embedding2):
    # 计算余弦相似度
    cos_sim = np.dot(embedding1, embedding2) / (np.linalg.norm(embedding1) * np.linalg.norm(embedding2))
    # 归一化到 [0, 1]
    similarity_score = (cos_sim + 1) / 2
    return similarity_score


# ---------------- Serial Manager ----------------
class SerialManager:
    def __init__(self, baudrate=9600, test_string="test\n", timeout=5):
        self.baudrate = baudrate
        self.test_string = test_string
        self.timeout = timeout
        self.port = None

    def find_device(self):
        ports = serial.tools.list_ports.comports()
        print(f"🔍 正在扫描 {len(ports)} 个串口设备...")
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        processes = []

        for port in ports:
            p = multiprocessing.Process(target=self._test_port, args=(port.device, return_dict))
            p.start()
            processes.append(p)

        start_time = time.time()
        while time.time() - start_time < self.timeout:
            if not any(p.is_alive() for p in processes): break
            time.sleep(0.1)

        for p in processes:
            if p.is_alive():
                p.terminate()
                p.join()

        for port in ports:
            if return_dict.get(port.device, None):
                print(f"✅ 找到 Digispark 设备：{port.device}")
                self.port = port.device
                return self.port

        print("❌ 没找到 Digispark")
        return None

    def _test_port(self, port_name, return_dict):
        try:
            ser = serial.Serial(port=port_name, baudrate=self.baudrate, timeout=0.5)
            time.sleep(1)
            ser.write(self.test_string.encode())
            time.sleep(0.3)
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode(errors='ignore').strip()
                if response.lower() == self.test_string.strip().lower():
                    return_dict[port_name] = response
            ser.close()
        except Exception:
            return_dict[port_name] = None

    def send(self, message="unlock\n", wait_time=0.5):
        if not self.port:
            print("⚠️ 串口未连接")
            return None
        try:
            ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=2)
            time.sleep(2)
            if not message.endswith('\n'):
                message += '\n'
            ser.write(message.encode())
            print(f"📤 发送指令：{repr(message)}")
            time.sleep(wait_time)
            response = ser.read_all().decode(errors='ignore')
            print(f"📥 收到回复: {response}")
            ser.close()
            return response
        except serial.SerialException as e:
            print(f"串口错误: {e}")
            return None

# ---------------- Face Recognizer ----------------
class FaceRecognizer:
    def __init__(self, reference_img_path, det_size=(640,480), threshold=0.7):
        self.threshold = threshold
        self.app = FaceAnalysis(name=["buffalo_l", "buffalo_m", "buffalo_s"][0], providers=["CPUExecutionProvider"])
        self.app.prepare(ctx_id=0, det_size=det_size)
        self.reference_embeddings = self.load_references(reference_img_path)
        if not self.reference_embeddings: raise ValueError("无法加载任何参考人脸特征")

    def load_references(self, dir_path):
        embeddings = []
        img_files = sorted(glob.glob(os.path.join(dir_path, '*')))
        print(f"🔍 加载参考图片目录: {dir_path}，共 {len(img_files)} 张图片")
        if not img_files:
            print("⚠️ 警告：目录为空！")
            return embeddings
        bar = tqdm(total=len(img_files), ncols=80)
        for img_path in img_files:
            img_name = os.path.splitext(os.path.basename(img_path))[0]
            bar.set_description(f"加载参考图片[{img_name[:30]}]: ")
            img = cv2.imread(img_path)
            if img is None:
                print(f"⚠️ 无法读取图片: {img_path}，跳过")
                bar.update(1)
                continue
            faces = self.app.get(img)
            if faces: embeddings.append((faces[0].embedding, img_name))
            else: print(f"⚠️ 警告：{img_path} 没有人脸，跳过")
            bar.update(1)
        bar.close()
        print(f"✅ 成功加载 {len(embeddings)} 张参考人脸图")
        return embeddings
    
    def recognize(self, frame):
        faces = self.app.get(frame)
        if not faces: return False, None, None
        face = faces[0]
        similarities = [
            (calculate_similarity(face.embedding, ref_emb), name)
            for ref_emb, name in self.reference_embeddings
        ]
        similarities.sort(reverse=True)  # 相似度高的排前面
        best_score, best_name = similarities[0]
        recognized = best_score > self.threshold
        return recognized, best_score, face.bbox, best_name


# ---------------- Main Logic ----------------
def monitor(reference_img_path, cooldown_sec=10, debug=False):
    serial_mgr = SerialManager()
    if not serial_mgr.find_device(): return
    recognizer = FaceRecognizer(reference_img_path)
    last_trigger_time = 0
    cap = None
    try:
        while True:
            if not is_locked_by_logonui():
                if cap:
                    print("🔓 检测到系统已解锁，释放摄像头")
                    cap.release()
                    cap = None
                    if debug: cv2.destroyAllWindows()
                time.sleep(1)
                continue
            if cap is None:
                print("🔄 锁屏检测到，打开摄像头")
                time.sleep(10)
                cap = cv2.VideoCapture(1)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ret, frame = cap.read()
            if not ret:
                print("摄像头读取失败")
                break
            recognized, similarity, bbox, matched_name = recognizer.recognize(frame)
            if bbox is not None:
                bbox = bbox.astype(int)
                label = f"{matched_name} {similarity:.2f}"
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 2)
                cv2.putText(frame, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            if recognized and (time.time() - last_trigger_time > cooldown_sec):
                print(f"✅ 识别成功！识别为: {matched_name}，相似度: {similarity:.2f}，触发 Digispark 解锁")
                serial_mgr.send("unlock")
                save_frame(frame)
                last_trigger_time = time.time()
            if debug:
                cv2.imshow("Debug View", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        if cap: cap.release()
        if debug: cv2.destroyAllWindows()
        print("🛑 摄像头已释放")
        
        
if __name__ == "__main__":
    # ---------------- debug ---------------------- #
    # print("🔧 进入摄像头调试模式...")
    # recognizer = FaceRecognizer(reference_img_path="./faces/")
    # cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # while True:
    #     ret, frame = cap.read()
    #     if not ret: break
    #     recognized, similarity, bbox, matched_name = recognizer.recognize(frame)
    #     if bbox is not None:
    #         bbox = bbox.astype(int)
    #         label = f"{matched_name} {similarity:.2f}"
    #         cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 2)
    #         cv2.putText(frame, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    #     if recognized:
    #         print(f"✅ 识别成功！识别为: {matched_name}，相似度: {similarity:.2f}")
    #     cv2.imshow("Camera Debug View", frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'): break
    # cap.release()
    # cv2.destroyAllWindows()
    # print("🛑 摄像头调试结束")
    # ---------------- debug ---------------------- #
    
    monitor(reference_img_path="./faces/", cooldown_sec=10, debug=False)


