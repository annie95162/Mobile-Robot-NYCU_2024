#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time

# 開啟相機
cap = cv2.VideoCapture('/dev/video0')  # 使用 USB 相機的設備名稱

# 等待相機初始化
time.sleep(2)

if not cap.isOpened():
    print("無法開啟相機")
    exit()

# 設定影像的寬和高
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # 讀取影像
    ret, frame = cap.read()
    if not ret:
        print("無法讀取相機影像")
        break

    # 顯示影像
    cv2.imshow("Camera Test", frame)

    # 按下 ESC 鍵退出
    k = cv2.waitKey(1)
    if k % 256 == 27:
        print("ESC 被按下，結束程序...")
        break

# 釋放相機並關閉視窗
cap.release()
cv2.destroyAllWindows()

