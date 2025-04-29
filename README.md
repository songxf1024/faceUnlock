# faceUnlock
Low-cost implementation of face recognition to unlock Windows.    
> Video Demo：[https://www.bilibili.com/video/BV1rD5mz9ETB/](https://www.bilibili.com/video/BV1rD5mz9ETB/)    
> Snapshot:    
> <p align="center"><img src="https://github.com/user-attachments/assets/4b315d68-0056-4b60-85f8-f165b188acd9" alt="image" width="400"/></p>


# Environments
## Hardware
- Digispark Attiny85
- USB-TTL Serial


## Software
- [Arduino](https://www.arduino.cc/en/Main/Software)
- [Python](https://www.python.org/downloads/)
- Opencv
- dlib
- pyserial


## ThirdParty
- [dlib feature](https://xfxuezhang.lanzouo.com/ibMSz2u5pjeb)
- [Digispark Driver](https://raw.githubusercontent.com/songxf1024/faceUnlock/refs/heads/main/thirdparty/Digistump.Drivers.zip)
- [package_digistump_index.json](https://raw.githubusercontent.com/songxf1024/faceUnlock/refs/heads/main/thirdparty/package_digistump_index.json)
- [zadig](https://raw.githubusercontent.com/songxf1024/faceUnlock/refs/heads/main/thirdparty/zadig-2.9.exe)
- [...](https://github.com/songxf1024/faceUnlock/tree/main/thirdparty)
- [VC_redist](https://learn.microsoft.com/zh-cn/cpp/windows/latest-supported-vc-redist?view=msvc-170)


# Usage
1. 使用 Arduino IDE 烧录 Digispark 的代码；
2. 将 Digispark 的 PB0(TX) 和 PB1(RX) 与 USB-TTL 连接；
3. 将 Digispark 和 USB-TTL 连接电脑；
4. 在faces目录下放目标人脸图片，可以放多张；
5. 在电脑上运行 listen.py ；
6. listen.py 检测到电脑锁屏后，会自动触发人脸识别，识别成功后调用 Digispark 进行解锁；


# Develop
```bash
conda create -n faceunlock python=3.9
conda activate faceunlock

pip install opencv-python pyserial onnx onnxruntime insightface psutil tqdm
```


# Blog
- [【教程】Digispark搭建开发环境和测试烧录](https://xfxuezhang.blog.csdn.net/article/details/147400007)
- [【教程】Digispark实现串口通信](https://xfxuezhang.blog.csdn.net/article/details/147404668)
- [【教程】ISP烧录Digispark的BootLoader固件](https://xfxuezhang.blog.csdn.net/article/details/147524103)


# TODO
- HID的输入内容(如解锁密码)由 listen.py 发送，而不是在 Digispark 中写死；
- 将 Digispark 和 USB-TTL 集成在一块电路板上；
- ...



