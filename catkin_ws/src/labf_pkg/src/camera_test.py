#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time

# �}�Ҭ۾�
cap = cv2.VideoCapture('/dev/video0')  # �ϥ� USB �۾����]�ƦW��

# ���ݬ۾���l��
time.sleep(2)

if not cap.isOpened():
    print("�L�k�}�Ҭ۾�")
    exit()

# �]�w�v�����e�M��
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # Ū���v��
    ret, frame = cap.read()
    if not ret:
        print("�L�kŪ���۾��v��")
        break

    # ��ܼv��
    cv2.imshow("Camera Test", frame)

    # ���U ESC ��h�X
    k = cv2.waitKey(1)
    if k % 256 == 27:
        print("ESC �Q���U�A�����{��...")
        break

# ����۾�����������
cap.release()
cv2.destroyAllWindows()

