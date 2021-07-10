#!/usr/bin/env python3

# referance: https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/lib_aruco_pose.py

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

horizontal_res = 1280
vertical_res = 720

horizontal_fov = 62.2 # degrees
vertical_fov = 48.8 # degrees

deg2rad = (math.pi/180)
rad2deg = 1/deg2rad
horizontal_fov_rad = horizontal_fov*deg2rad
vertical_fov_rad = vertical_fov*deg2rad

class ArucoTracker():
    def __init__(self,
                camera_matrix,
                camera_distortion):

        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion

        #--- Rotation matrix for rotation around x axis
        self._R_flip = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1
        self._R_flip[1,1] =-1.0
        self._R_flip[2,2] =-1.0

        #--- define aruco dict
        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self._parameters  = aruco.DetectorParameters_create()

        #--- Font for image text
        self.font = cv2.FONT_HERSHEY_PLAIN

    def track(self, frame, id_to_find, marker_size, alt, verbose=False, show_video=False):
        x = y = z = 0.0
        x_error_angle = y_error_angle = 0
        marker_found = False


        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray,
                        dictionary=self._aruco_dict, 
                        parameters=self._parameters)

        
        if ids is not None and id_to_find in ids:
            marker_found = True
            id_index = -1
            for i in range(len(ids)):
                if id_to_find == ids[i]: id_index = i

            # tvec, rvec, _objPoints = aruco.estimatePoseSingleMarkers([corners[id_index]], marker_size, self._camera_matrix, self._camera_distortion)
            # tvec, rvec = tvec[0,0,:], rvec[0,0,:]
            # x, y, z = tvec[0], tvec[1], tvec[2]
            
            # the corners are in mm as the camera was caliberated with mm as the input
            marker_corners = corners[id_index]
            x_sum = marker_corners[0][0][0] + marker_corners[0][1][0] + marker_corners[0][2][0] +marker_corners[0][3][0]
            y_sum = marker_corners[0][0][1] + marker_corners[0][1][1] + marker_corners[0][2][1] +marker_corners[0][3][1]

            x_avg = x_sum*0.25 #px
            y_avg = y_sum*0.25 #px

            x_error = x_avg - horizontal_res*0.5 #px
            y_error = vertical_res*0.5 - y_avg   #px

            x_error_angle = x_error*horizontal_fov_rad/horizontal_res #rad
            y_error_angle = -y_error*vertical_fov_rad/vertical_res     #rad

            x = x_avg
            y = y_avg

            alt = alt*1000+3.04 #mm

            x = alt*math.tan(x_error_angle)
            y = alt*math.tan(y_error_angle)
           
            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, [corners[id_index]], borderColor=[255, 0, 0])
            # aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

            str_position_px = "MARKER Position x=%4.0fpx  y=%4.0fpx"%(x_error, y_error)
            cv2.putText(frame, str_position_px, (0, 100), self.font, 3, (0, 255, 0), 2, cv2.LINE_AA)

            str_position_deg = "MARKER Position x=%4.0fdeg y=%4.0fdeg"%(x_error_angle*rad2deg, y_error_angle*rad2deg)
            cv2.putText(frame, str_position_deg, (0, 200), self.font, 3, (0, 255, 0), 2, cv2.LINE_AA)

            str_position_mm = "MARKER Position x=%4.0fmm  y=%4.0fmm"%(x, y)
            cv2.putText(frame, str_position_mm, (0, 300), self.font, 3, (0, 255, 0), 2, cv2.LINE_AA)

            if verbose: print("Marker id=%d X = %.1fmm  Y = %.1fmm"%(id_to_find, x_avg, y_avg))

        return(marker_found, frame, x*0.001, y*0.001, x_error_angle, y_error_angle, corners)

    
    def _rotationMatrixToEulerAngles(self,R):
        # Calculates rotation matrix to euler angles
        # The result is the same as MATLAB except the order
        # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])