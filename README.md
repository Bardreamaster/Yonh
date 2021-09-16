# Yonh - you only need hands
You only need a cube in your hands to control almost all robots.


## Introduction
Position and speed control of robotic arms or other electromechanical systems using low-cost solutions.
    
## Files Structure  
``` 
├── checkBoard_calib.py   # calibration for camera using images of checkerboard  
├── config  
│   ├── calib60.yaml  
│   ├── robot.proto  
│   └── ur10e.urdf  
├── gripper.py  
├── keyboard_test.py  
├── pid.py  
├── README.md  
├── requirements.txt  
├── singlePicture_capture.py    # singlePicture_capture.py
├── staticDataCollect.py  
├── test                        # [README.md](test/README.md)  
│   ├── gripper_test.py  
│   ├── interface-test.ipynb  
│   ├── README.md  
│   ├── test_pid.py  
│   └── ur_test.py  
├── UR_control.py  
└── xyz_control.py  
```
## Demonstration 

Bilibili：  
[![Demo](https://res.cloudinary.com/marcomontalbano/image/upload/v1631803649/video_to_markdown/images/youtube--x4njxymneQA-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.bilibili.com/video/BV1yM4y1V73B?spm_id_from=333.999.0.0 "")  

YouTube：
[![Demo](https://res.cloudinary.com/marcomontalbano/image/upload/v1631803649/video_to_markdown/images/youtube--x4njxymneQA-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=x4njxymneQA "")