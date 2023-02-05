# vision_script.py

import sim
import time
import cv2 as cv
import numpy as np

# from matplotlib import pyplot as plt

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
            out_img = image
            # sim.simxSetVisionSensorImage(clientID, v1, out_img, 0, sim.simx_opmode_oneshot)
            # print('detectionState =', p_data[1], ' detectedPoint =', p_data[2], ' detectedObjectHandle =',
            # p_data[3], ' detectedSurfaceNormalVector =', p_data[4])
            if p_data[1]:
                #sim.simxSynchronous(clientID, True)
                #sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

                img = np.array(image).astype(np.uint8)
                img = img.reshape(resolution[0], resolution[1], 3).transpose([1, 0, 2])
                img_copy = img.copy()
                gray = cv.cvtColor(img_copy, cv.COLOR_BGR2GRAY)
                _, threshold = cv.threshold(gray, 145, 255, cv.THRESH_BINARY)
                contours, _ = cv.findContours(threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                i = 0

                for contour in contours:
                    if i == 0:
                        i = 1
                        continue

                    approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True)
                    cv.drawContours(img_copy, [contour], 0, (0, 0, 255), 5)

                    # finding center point of shape
                    M = cv.moments(contour)
                    if M['m00'] != 0.0:
                        x = int(M['m10'] / M['m00'])
                        y = int(M['m01'] / M['m00'])

                    # putting shape name at center of each shape
                    if len(approx) == 3:
                        cv.putText(img_copy, 'TRIANGLE', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    elif len(approx) == 4:
                        cv.putText(img_copy, 'QUAD', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    elif len(approx) == 5:
                        cv.putText(img_copy, 'PENTA', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    elif len(approx) == 6:
                        cv.putText(img_copy, 'HEXA', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    else:
                        cv.putText(img_copy, 'CIRCLE', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                    out_img = img_copy.transpose([1, 0, 2]).reshape(resolution[0] * resolution[1] * 3).tolist()

            sim.simxSetVisionSensorImage(clientID, v1, out_img, 0, sim.simx_opmode_oneshot)
            #sim.simxSynchronousTrigger(clientID)
            #sim.simxGetPingTime(clientID)
            #sim.simxSynchronous(clientID, False)
        elif ret == sim.simx_return_novalue_flag:
            print("no image yet")
        else:
            print(ret, '-', p_data[0])
else:
    print("Failed to connect to remote API Server")
    sim.simxFinish(clientID)
