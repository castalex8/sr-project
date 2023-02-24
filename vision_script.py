import sim
import time
import cv2 as cv
import numpy as np

def shapeDetection(resolution, image):
    shape = 'NONE'
    img = np.array(image).astype(np.uint8)
    img = img.reshape(resolution[0], resolution[1], 3).transpose([1, 0, 2])
    img_copy = img.copy()
    gray = cv.cvtColor(img_copy, cv.COLOR_BGR2GRAY)
    _, threshold = cv.threshold(gray, 145, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True)
        cv.drawContours(img_copy, [contour], 0, (0, 0, 255), 5)
        x = 0
        y = 0
        # finding center point of shape
        M = cv.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])

        # putting shape name at center of each shape
        if len(approx) == 3:
            shape = 'TRIANGLE'
            cv.putText(img_copy, 'TRIANGLE', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        elif len(approx) == 4:
            shape = 'QUAD'
            cv.putText(img_copy, 'QUAD', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        elif len(approx) == 5:
            shape = 'PENTA'
            cv.putText(img_copy, 'PENTA', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        elif len(approx) == 6:
            shape = 'HEXA'
            cv.putText(img_copy, 'HEXA', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        else:
            shape = 'CIRCLE'
            cv.putText(img_copy, 'CIRCLE', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    out_img = img_copy.transpose([1, 0, 2]).reshape(resolution[0] * resolution[1] * 3).tolist()
    return out_img, shape

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19990, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
    # get handles
    _, vision0 = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    _, vision1 = sim.simxGetObjectHandle(clientID, 'Vision_sensor2', sim.simx_opmode_blocking)
    _, conn = sim.simxGetObjectHandle(clientID, '/connection', sim.simx_opmode_blocking)
    _, prox = sim.simxGetObjectHandle(clientID, '/Main/conveyor/Proximity_sensor', sim.simx_opmode_blocking)
    vision0_data = sim.simxGetVisionSensorImage(clientID, vision0, 0, sim.simx_opmode_streaming)
    prox_data = sim.simxReadProximitySensor(clientID, prox, sim.simx_opmode_streaming)
    # conn_data = sim.simxReadForceSensor(clientID, conn, sim.simx_opmode_streaming)
    time.sleep(1)
    flag = 0

    # while there's connection
    while sim.simxGetConnectionId(clientID) != -1:
        prox_data = sim.simxReadProximitySensor(clientID, prox, sim.simx_opmode_buffer)
        # if prox works
        if prox_data[0] == sim.simx_return_ok:
            # if prox detects
            if prox_data[1]:
                # conn_code, conn_state, force, torque = sim.simxReadForceSensor(clientID, conn, sim.simx_opmode_buffer)
                vision0_ret, resolution, image = sim.simxGetVisionSensorImage(clientID, vision0, 0, sim.simx_opmode_buffer)
                # if
                if vision0_ret == sim.simx_return_ok:
                    if flag == 0:
                        flag = 1  # da settare con --> string signalName=sim.getSignalName(int signalIndex,
                        # int signalType)
                        out_img, shape = shapeDetection(resolution, image)
                        sim.simxSetVisionSensorImage(clientID, vision1, out_img, 0, sim.simx_opmode_oneshot)
                        sim.simxSetStringSignal(clientID, "shape_detected", shape, sim.simx_opmode_oneshot)
                        # print(force[2])
                        # 9.81 forza g \ 0.4 peso della mano \ 1.08 fattore fisso
                        #print(force[2] / 9.81 - 0.4 - 1.095)

                elif vision0_ret == sim.simx_return_novalue_flag:
                    print("no image yet")
                # elif conn_code == sim.simx_return_novalue_flag:
                    # print("error on force sensor")
                else:
                    print(vision0_ret, '-', prox_data[0])
else:
    print("Failed to connect to remote API Server")
    sim.simxFinish(clientID)
