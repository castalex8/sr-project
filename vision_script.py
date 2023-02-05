# vision_script.py
#
# Demo of simple image retranslate from v0 to v1

import sim
import time
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1', 19990, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    # get handles
    _, v0 = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    _, v1 = sim.simxGetObjectHandle(clientID, 'Vision_sensor2', sim.simx_opmode_blocking)
    _, p = sim.simxGetObjectHandle(clientID, '/Main/conveyor/Proximity_sensor', sim.simx_opmode_blocking)

    v0_data = sim.simxGetVisionSensorImage(clientID, v0, 0, sim.simx_opmode_streaming)
    p_data = sim.simxReadProximitySensor(clientID, p, sim.simx_opmode_streaming)
    time.sleep(1)

    while sim.simxGetConnectionId(clientID) != -1:
        ret, resolution, image = sim.simxGetVisionSensorImage(clientID, v0, 0, sim.simx_opmode_buffer)
        p_data = sim.simxReadProximitySensor(clientID, p, sim.simx_opmode_buffer)

        if ret + p_data[0] == 0:
            sim.simxSetVisionSensorImage(clientID, v1, image, 0, sim.simx_opmode_oneshot)
            # print('detectionState =', p_data[1], ' detectedPoint =', p_data[2], ' detectedObjectHandle =',
            # p_data[3], ' detectedSurfaceNormalVector =', p_data[4])
            if p_data[1] == True:
                img = np.array(image).astype(np.uint8)
                img = img.reshape(resolution[0], resolution[1], 3).transpose([1, 0, 2])

        elif ret == sim.simx_return_novalue_flag:
            print("no image yet")
        else:
            print(ret, '-', p_data[0])
else:
    print("Failed to connect to remote API Server")
    sim.simxFinish(clientID)
