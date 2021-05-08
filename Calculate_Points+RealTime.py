#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 18:41:10 2020

@author: richard
"""
# --- standard imports ---
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from numpy import matlib
from rigidtrans import rigidtrans_func
import time
import cv2
import sys 
# --- qtm imports ---
import asyncio
import qtm
import time
import datetime
from qtm.packet import QRTComponentType
# --- change to local directory ---
import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

noPoints = 9
Q_vec = np.zeros((noPoints, 3))

t_act = time.time()


# %matplotlib qt
def calculateFramePosition():
    # Calculates the Rigid Body Transformation based on the measured points
    # open measurement_pts.mat

    data = pd.read_csv('measurement_pts.csv')

    x = data['x']
    y = data['y']
    z = data['z']

    # position body fixed frame in the middle of the 4 rotor points
    # rotor points are (2), (3), (7) and (9)
    x0 = np.mean([x[1], x[2], x[6], x[8]])
    y0 = np.mean([y[1], y[2], y[6], y[8]])
    z0 = np.mean([z[1], z[2], z[6], z[8]]) -70 # minus 70 because tracking points are on the propeller guard

    # calculate vectors to each point w.r.t. the new center
    I_vec = np.zeros((noPoints, 3))
    for i in range(noPoints):
        I_vec[i, 0] = x[i] - x0
        I_vec[i, 1] = y[i] - y0
        I_vec[i, 2] = z[i] - z0

    # point (4) is pointing in the positive +x direction
    # calculate angle between reference frame and body frame
    alpha = np.arctan2(y[3]-y0, x[3]-x0)
    # print(alpha*180/np.pi)

    # transformation matrix, positive rotation around z-axis
    R_QI = np.array([[np.cos(alpha), +np.sin(alpha), 0],
                     [-np.sin(alpha), np.cos(alpha), 0], [0, 0, 1]])
    R_QI = np.matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]]
                     ) @ R_QI  # reassign axis

    # rotate vectors into body-fixed frame
    for i in range(noPoints):
        Q_vec[i, :] = R_QI @ I_vec[i, :].T

    # body frame coordinates
    I_cx = np.array([1, 0, 0])
    I_cy = np.array([0, 1, 0])
    I_cz = np.array([0, 0, 1])

    Q_cx = (R_QI.dot(I_cx.T)).T
    Q_cy = (R_QI.dot(I_cy.T)).T
    Q_cz = (R_QI.dot(I_cz.T)).T

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(x0, y0, z0, 'x')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # plot something
    if False: 
        # plot connecting lines to frame
        if True:
            for i in range(noPoints):
                ax.plot3D([x0, x0+I_vec[i, 0]], [y0, y0+I_vec[i, 1]],
                        [z0, z0+I_vec[i, 2]], 'k')

        # plot connecting lines for all points
        if True:
            for i in range(noPoints):
                for k in range(noPoints):
                    if i != k:
                        ax.plot3D([x0+I_vec[i, 0], x0+I_vec[k, 0]],
                                [y0+I_vec[i, 1], y0+I_vec[k, 1]], [z0+I_vec[i, 2], z0+I_vec[k, 2]], 'c')

        # plot body-fixed frame
        ax.plot3D([x0, x0+Q_cx[0,0]*50], [y0, y0+Q_cx[1,0]*50],
                [z0, z0+Q_cx[2,0]*50], 'r', linewidth=2)
        ax.plot3D([x0, x0+Q_cy[0,0]*50], [y0, y0+Q_cy[1,0]*50],
                [z0, z0+Q_cy[2,0]*50], 'g', linewidth=2)
        ax.plot3D([x0, x0+Q_cz[0,0]*50], [y0, y0+Q_cz[1,0]*50],
                [z0, z0+Q_cz[2,0]*50], 'b', linewidth=2)

        # show resulting plot
        plt.show()

        # check if rigidtrans_func works as expected
        [R_QP, Q_t_PQ, Q_r_P] = rigidtrans_func(I_vec, Q_vec, np.ones(noPoints))

        # result is e-16, so it works as expected
        print(R_QP - R_QI)
        print(Q_t_PQ)

# Calculate body fixed frame from measurement data


def on_packet(packet):
    """ Callback function that is called everytime a data packet arrives from QTM """

    # only handle packets in a set interval
    global t_act
    t = time.time()
    if t > t_act + 0.05:  # 0.05 = 50ms
        interval = t - t_act 
        t_act = t
        if QRTComponentType.Component3d in packet.components:
            header, markers = packet.get_3d_markers()
        meas_vec = np.zeros((9, 3))

        t = time.time()  # 0.003192 s per frame on laptop --> Raspberry Pi 4 ~ 16ms while openCV runs
        for i in range(9):
            meas_vec[i, :] = [markers[i][0], markers[i][1], markers[i][2]]

        [R_QP, Q_t_PQ, Q_r_P] = rigidtrans_func(
            meas_vec, Q_vec[0:9], np.ones(9))


        
        # euler angles of rotation matrix
        R_IL = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_ = R_IL @ R_QP

        euler = np.zeros(3)

        euler[0] = np.arctan2(R_[1, 2], R_[2, 2])
        
        euler[2] = np.arctan2(-R_[0, 1], R_[0, 0])

        euler[1] = np.arctan2(R_[0, 2], R_[0, 0]/np.cos(euler[2]))

        # Creating a gray image with 3 x
        # channels RGB and unsigned int datatype
        img = 200 * np.ones((500, 500, 3), np.uint8)
        R_ScreenI = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        centerx = 250
        centery = 250
        cCenter = (centerx, centery)

        # coordinates base vectors
        I_cx = np.array([1, 0, 0])
        I_cy = np.array([0, 1, 0])
        I_cz = np.array([0, 0, 1])

        # transform into opencv screen coordinates  →x
        #                                          ↓y

        # TODO: does this work??
        R_SP = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) @ R_QP

        S_cx = R_SP @ I_cx
        S_cy = R_SP @ I_cy
        S_cz = R_SP @ I_cz

        centerx = 250
        centery = 250
        cCenter = (centerx, centery)
        # draw coordinate frame
        cv2.line(img, cCenter, (centerx+int(100 *
                                            S_cx[0]), centery+int(100*S_cx[1])), (0, 0, 255), 4)  # x
        cv2.line(img, cCenter, (centerx+20, centery),
                 (0, 0, 255), 1)  # x-fixed
        cv2.line(img, cCenter, (centerx+int(100 *
                                            S_cy[0]), centery+int(100*S_cy[1])), (0, 255, 0), 4)  # y
        cv2.line(img, cCenter, (centerx, centery+20),
                 (0, 255, 0), 1)  # y-fixed
        cv2.line(img, cCenter, (centerx+int(100 *
                                            S_cz[0]), centery+int(100*S_cz[1])), (255, 0, 0), 4)  # z
        cv2.line(img, cCenter, (centerx, centery), (255, 0, 0), 1)  # z-fixed

        cv2.putText(img, 'X/Y/Z:', (20, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, '{} '.format(Q_t_PQ), (5, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, '{} '.format(euler*180.0/np.pi), (5, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.putText(img, 'FPS: {:0.2f} '.format(1/interval), (5, 480),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)


        print("{},{}".format(Q_t_PQ, euler))
        sys.stdout.flush()
        cv2.imshow('Quadrotor State', img)

        #time.sleep((1/24) - 0.001)
        wK = cv2.waitKey(1)
        if wK & 0xFF == ord('q'):
            cv2.destroyAllWindows()
        elif wK & 0xFF == ord('k'):
            cv2.destroyAllWindows()
            sys.exit()


async def setup():
    """ Main function """
    connection = await qtm.connect("192.168.137.1")
    if connection is None:
        return

    await connection.stream_frames(components=["3d"], on_packet=on_packet)


if __name__ == "__main__":
    calculateFramePosition()
    asyncio.ensure_future(setup())
    asyncio.get_event_loop().run_forever()
