# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 15:40:01 2020

@author: 54252
"""

import cv2
import os
import numpy as np
import paho.mqtt.subscribe as subscribe

def receiveMqtt():
    msg = subscribe.simple("test", hostname="192.168.1.120")
    msg = str(msg.payload)
    msg_list = msg.split(sep=",")
    msg_list = msg_list[1:193]
    piexls = []
    for i in msg_list:
        piexls.append(float(i))
    piexls1 = np.array(piexls)
    piexls1.resize(12, 16)
    piexls1 = piexls1[:, 5 : 11]
    return piexls1

def load_image(fpath):
    images = []
    
    filenames = os.listdir(fpath)
    
    for filename in filenames:
        images.append(cv2.imread(fpath + r'/' + filename, cv2.IMREAD_GRAYSCALE))
    
    return images

def find_center(image):
    image  = np.resize(image, [8, 8]) #将长度为64的像素值转换为8x8大小
    image =  np.round(image).astype(np.uint8) #转换数据格式
    image = cv2.medianBlur(image, 3)  #中值滤波
    _, image = cv2.threshold(image, 22, 1, cv2.THRESH_BINARY) #二值化  阈值为21.5
    
    rows = np.zeros(8)
    cols = np.zeros(8)
    
    for i in range(8):
        for j in range(8):
            if(image[i][j] == 1):
                rows[i] += 1
                cols[j] += 1
                
    row = np.mean(np.argwhere(rows == np.max(rows)))
    col = np.mean(np.argwhere(cols == np.max(cols)))
    
    return row, col

def isPassing(image, threshold):
    tmax = np.max(image)
    avg = np.average(image)
    median = np.median(image)
    
    return tmax - median > threshold