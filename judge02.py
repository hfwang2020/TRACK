# -*- coding: utf-8 -*-
"""
Created on Tue Oct 20 15:44:36 2020

@author: 54252
"""

import utils as ut
import numpy as np
from medianBlur import medianBlur, binary, binaryCenter
import time
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe('data')

# 消息接收回调
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client(client_id="6b4deb575dc7e791470fb2c72a5432c0")

# 指定回调函数
client.on_connect = on_connect
client.on_message = on_message

# 建立连接
client.connect('bemfa.com', 9501, 60)


count = 8
images = []
ite = 0
#
#rowss = []
#colss = []
threshold = 1.4
while(True):
    print('ite:%d'%(ite))        
    ite += 1
    
    image = ut.receiveMqtt()
    image = medianBlur(image)
    images.append(image)
    
    mid = np.median(image)
    max = np.max(image)
#    avg = np.average(image)
    print('mid = %f, max = %f, max - mid = %f' % (mid, max, max - mid))
#    print('avg = %f, max = %f, max - avg = %f' % (avg, max, max - avg))
    
    if(ut.isPassing(image, threshold)):
        mid = np.median(image)
        
        rows = []
        cols = []
        
        image = binary(image, mid + threshold)
        row, col = binaryCenter(image)
        rows.append(row)
        cols.append(col)
#        rowss.append(row)
#        colss.append(col)
        
        while(True):
            print('ite:%d'%(ite))
            ite += 1
            
            nextImage = ut.receiveMqtt()
            nextImage = medianBlur(nextImage)
            images.append(nextImage)
            
            mid = np.median(nextImage)
            max = np.max(nextImage)
#            avg = np.average(nextImage)
            print('mid = %f, max = %f, max - mid = %f' % (mid, max, max - mid))
#            print('avg = %f, max = %f, max - avg = %f' % (avg, max, max - avg))
            
            if(not ut.isPassing(nextImage, threshold)):
                break
            
            nextImage = binary(nextImage, mid + threshold)
            row, col = binaryCenter(nextImage)
            rows.append(row)
            cols.append(col)
#            rowss.append(row)
#            colss.append(col)
            
            time.sleep(0.08)
        
#        nums = int(len(cols) / 20) + 1 #单次出入的人数
        
        if(0 <= cols[-1] <= 1.5):
            count += 1
            print('有1人进入，当前人数为%d'%(count))

        
        if(4.5 <= cols[-1] <= 6):
            count -= 1
            print('有1人离开，当前人数为%d'%(count))
            
        
        client.publish('data01',payload=str(count),qos=0)
   
#    image = binary(image, mid + threshold)
#    row, col = binaryCenter(image)
#    rowss.append(row)
#    colss.append(col)
    
    time.sleep(0.1)

#np.save('sample01.npy', images)
            
    

