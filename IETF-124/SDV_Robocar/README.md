## Main steps
**LIMO**<br/>
- Run server<br/>
- Stream camera<br/>
- Scan LiDAR
- Subscribe to /wheel/odom topic(from ```base_limo.launch.py```)<br/>
- Subscribe to /scan topic(from ```ydlidar.launch.py```)<br/>
+ Receive distance of closest person from Desktop to control the LIMO<br/>

**Desktop**<br/>
- Receive camera stream from LIMO<br/>
- Extract class and bbox information from LIMO<br/>
- Receive odometry data(pose, twist) from LIMO and compute GPS coordinates<br/>
- Receive 2D LiDAR data(angle_min, angle_increment, ranges) from LIMO and compute Camera & LiDAR calibration<br/>
- POST distance to LIMO<br/>


**Kubernetes**<br/>
- Run server<br/>
- Save JSON files & Images in real time (1s)<br/>
<br/>

### 1. Run IMO server (LIMO)
```
$ ros2 launch limo_base limo_base.launch.py    # terminal 1
$ ros2 launch ydlidar_ros2_driver ydlidar.launch.py   # terminal 2
$ python imo_server_lidar.py    # terminal 3
```

### 2. Run k8s server (Kubernetes)
```
$ python k8s_server.py
```

### 3. YOLO Detection + Odometry + 2D Lidar (Desktop)
```
$ python edge_control.py
```

### 4. Control IMO (LIMO)
```
$ python imo_control.py    # terminal 4
```

<br/>

- YOLO Detection (Desktop)<br/>
<img src="https://github.com/user-attachments/assets/cfd2f5f8-caf9-4f42-a71e-4700724dfdeb" width="500" height="450"/>
</br></br>

- Save files in real time (Kubernetes)</br>
<img src="https://github.com/user-attachments/assets/1af6d34e-9590-4214-96c4-a6921b0f2eea" width="500" height="350"/>
<img src="https://github.com/user-attachments/assets/9a6a7beb-29b9-4d06-8f85-fc2fe75a017a" width="500" height="350"/>

<br/><br/>

## Data classes
**yolov8s :**<br/>
{ 0: 'person',
 1: 'bicycle',
 2: 'car',
 3: 'motorcycle',
 4: 'airplane',
 5: 'bus',
 6: 'train',
 7: 'truck',
 8: 'boat',
 9: 'traffic light',
 10: 'fire hydrant',
 11: 'stop sign',
 12: 'parking meter',
 13: 'bench',
 14: 'bird',
 15: 'cat',
 16: 'dog',
 17: 'horse',
 18: 'sheep',
 19: 'cow',
 20: 'elephant',
 21: 'bear',
 22: 'zebra',
 23: 'giraffe',
 24: 'backpack',
 25: 'umbrella',
 26: 'handbag',
 27: 'tie',
 28: 'suitcase',
 29: 'frisbee',
 30: 'skis',
 31: 'snowboard',
 32: 'sports ball',
 33: 'kite',
 34: 'baseball bat',
 35: 'baseball glove',
 36: 'skateboard',
 37: 'surfboard',
 38: 'tennis racket',
 39: 'bottle',
 40: 'wine glass',
 41: 'cup',
 42: 'fork',
 43: 'knife',
 44: 'spoon',
 45: 'bowl',
 46: 'banana',
 47: 'apple',
 48: 'sandwich',
 49: 'orange',
 50: 'broccoli',
 51: 'carrot',
 52: 'hot dog',
 53: 'pizza',
 54: 'donut',
 55: 'cake',
 56: 'chair',
 57: 'couch',
 58: 'potted plant',
 59: 'bed',
 60: 'dining table',
 61: 'toilet',
 62: 'tv',
 63: 'laptop',
 64: 'mouse',
 65: 'remote',
 66: 'keyboard',
 67: 'cell phone',
 68: 'microwave',
 69: 'oven',
 70: 'toaster',
 71: 'sink',
 72: 'refrigerator',
 73: 'book',
 74: 'clock',
 75: 'vase',
 76: 'scissors',
 77: 'teddy bear',
 78: 'hair drier',
 79: 'toothbrush'
 }
 <br/><br/>
 **fine-tuned model (best.pt) :**<br/>
{ 0: 'counter',
 1: 'pillar',
 2: 'desk',
 3: 'ticket machine',
 4: 'fire extinguisher',
 5: 'wastebasket',
 6: 'information board',
 7: 'chair',
 8: 'car',
 9: 'sculpture',
 10: 'display stand',
 11: 'table',
 12: 'fence',
 13: 'sign',
 14: 'person'
 }
 
