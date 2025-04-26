import os
import serial
import serial.tools.list_ports
import time
import cv2
import dlib
import numpy as np
import psutil
from datetime import datetime
import multiprocessing

def is_locked_by_logonui():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == "LogonUI.exe": return True
    return False


def find_digispark(baudrate=9600, test_string="test\n", timeout_per_port=2):
    def test_port_worker(port_name, baudrate, test_string, return_dict):
        try:
            ser = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.5)
            time.sleep(1)
            ser.reset_input_buffer()
            ser.write(test_string.encode('utf-8'))
            time.sleep(0.3)
            response = ''
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            ser.close()
            return_dict[port_name] = response
        except Exception:
            return_dict[port_name] = None
    ports = serial.tools.list_ports.comports()
    print(f"🔍 正在并行扫描 {len(ports)} 个串口设备...")
    manager = multiprocessing.Manager()
    return_dict = manager.dict()
    processes = []
    # 给每个端口启动一个子进程
    for port in ports:
        port_name = port.device
        print(f"准备测试 {port_name}...")
        p = multiprocessing.Process(target=test_port_worker, args=(port_name, baudrate, test_string, return_dict))
        processes.append(p)
        p.start()
    # 等待所有子进程结束（统一超时控制）
    start_time = time.time()
    while time.time() - start_time < timeout_per_port:
        alive = any(p.is_alive() for p in processes)
        if not alive: break
        time.sleep(0.1)
    # 杀掉仍然活着的子进程
    for p in processes:
        if p.is_alive():
            print(f"⚠️ 超时终止子进程 {p.name}")
            p.terminate()
            p.join()
    # 检查结果
    for port in ports:
        port_name = port.device
        response = return_dict.get(port_name, None)
        if response:
            print(f"收到 {port_name} 的回显：{repr(response)}")
            if response.lower() == test_string.strip().lower():
                print(f"✅ 找到 Digispark！端口：{port_name}")
                return port_name
        else:
            print(f"⚠️ {port_name} 无响应或异常")
    print("❌ 没找到Digispark设备")
    return None

def send_to_digispark(port='COM3', baudrate=9600, send_string='u\n', wait_time=0.5, timeout=2):
    """
    向指定串口发送一个字符串，并接收返回内容。

    参数:
    - port: 串口端口 (例如 'COM3')
    - baudrate: 波特率，默认为 9600
    - send_string: 要发送的完整字符串（建议带换行符）
    - wait_time: 发送后等待响应的时间（秒）
    - timeout: 串口超时时间

    返回:
    - Digispark 的串口回复内容（字符串）
    """
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)  # 等待串口初始化

        # 发送字符串
        if not send_string.endswith('\n'):
            send_string += '\n'  # 自动补\n，防止漏掉
        ser.write(send_string.encode('utf-8'))
        print(f"已发送字符串：{repr(send_string)}")

        # 等待 Digispark 处理并回传
        time.sleep(wait_time)

        response = ser.read_all().decode(errors='ignore')
        print("收到回复：", response)

        ser.close()
        return response

    except serial.SerialException as e:
        print("串口错误:", e)
        return None


def get_face_embedding(image, detector, sp, model):
    dets = detector(image, 1)
    if len(dets) == 0:
        return None, None
    shape = sp(image, dets[0])
    face_descriptor = model.compute_face_descriptor(image, shape)
    return np.array(face_descriptor), dets[0]  # 返回人脸特征和矩形框

# 加载多图参考特征
def load_reference_embeddings(image_paths, detector, sp, model):
    embeddings = []
    for path in image_paths:
        img = cv2.imread(path)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        embedding, _ = get_face_embedding(rgb, detector, sp, model)
        if embedding is not None:
            embeddings.append(embedding)
    return embeddings

def save_recognized_face(frame):
    folder = "recognized_faces"
    os.makedirs(folder, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(folder, f"{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"📸 已保存识别图像到: {filename}")

def monitor_face_dlib(reference_img_path, cooldown_sec=10, port='COM3', debug=False):
    detector = dlib.get_frontal_face_detector()
    sp = dlib.shape_predictor("features/shape_predictor_68_face_landmarks.dat")
    model = dlib.face_recognition_model_v1("features/dlib_face_recognition_resnet_model_v1.dat")
    ref_img = cv2.imread(reference_img_path)
    ref_rgb = cv2.cvtColor(ref_img, cv2.COLOR_BGR2RGB)
    ref_embedding, _ = get_face_embedding(ref_rgb, detector, sp, model)
    if ref_embedding is None:
        print("❌ 无法提取参考图像人脸特征")
        return
    print("🔍 启动 dlib 人脸识别监控（带调试画面）" if debug else "🔍 启动 dlib 人脸识别监控（无界面）")
    last_trigger_time = 0
    cap = None
    try:
        while True:
            if not is_locked_by_logonui(): 
                # 系统已解锁
                if cap: 
                    print("🔓 检测到系统已解锁，释放摄像头")
                    cap.release()
                    cap = None
                    if debug: cv2.destroyAllWindows()
                time.sleep(1)
                continue
            
            # 系统是锁屏状态
            if cap is None:
                print("🔄 检测到锁屏，打开摄像头")
                cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            if not ret:
                print("摄像头读取失败")
                break
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            embedding, rect = get_face_embedding(rgb, detector, sp, model)
            if embedding is not None:
                dist = np.linalg.norm(ref_embedding - embedding)
                if debug and rect:
                    # 画出人脸框
                    left, top, right, bottom = rect.left(), rect.top(), rect.right(), rect.bottom()
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    cv2.putText(frame, f"dist={dist:.2f}", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                if dist < 0.6 and time.time() - last_trigger_time > cooldown_sec:
                    print(f"✅ 人脸识别成功（距离 {dist:.2f}），触发 Digispark")
                    send_to_digispark(send_string="unlock", port=port)
                    save_recognized_face(frame)
                    last_trigger_time = time.time()
            if debug:
                cv2.imshow("Face Debug View", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(0.1)
    finally:
        cap.release()
        if debug: cv2.destroyAllWindows()
        print("🛑 摄像头已释放")


if __name__ == "__main__":
    digispark_port = find_digispark()
    if digispark_port:
        monitor_face_dlib(
            reference_img_path="faces/me.jpg",
            cooldown_sec=10,
            port=digispark_port,
            debug=False
        )
    else:
        print("没有找到 Digispark，退出程序")
