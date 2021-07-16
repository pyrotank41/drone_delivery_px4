import cv2
import asyncio
import lib.delivery_lib
from lib.aruco_tracker_lib import *
import rospy
import numpy as np
import ros_numpy as rnp
from sensor_msgs.msg import Image


# camera parameters
_camera_matrix = np.array([[1.28779477e+03, 0.00000000e+00, 6.59268320e+02],
                        [0.00000000e+00, 1.28791404e+03, 3.64676675e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
_camera_distortion = np.array([0.15085, 0.0317, -0.00131, 0.00199, -0.99343])
_aruco_tracker = ArucoTracker(_camera_matrix, _camera_distortion)

horizontal_len = 1280 # px
virticle_len = 720    # px

horizontal_fov = 62.2 # degrees
vertical_fov = 48.8 # degrees

_aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
_parameters  = aruco.DetectorParameters_create()

_aruco_img_pub = rospy.Publisher('/landing/color/aruco_image', Image, queue_size=10)

def callback(message):

    np_img = rnp.numpify(message)
    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=np_img,
                    dictionary=_aruco_dict, 
                    parameters=_parameters)
    if ids is None:
        ids = []

    if len(ids) != 0:

        ret = aruco.estimatePoseSingleMarkers(corners, 14, _camera_matrix, _camera_distortion)
        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
        
        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(np_img, corners, borderColor=[0, 255, 0])
        aruco.drawAxis(np_img,_camera_matrix, _camera_distortion, rvec, tvec,10)

    im_rgb = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
    _aruco_img_pub.publish(rnp.msgify(Image, im_rgb, encoding='rgb8'))
    print(ids)




if __name__ == "__main__":
    rospy.init_node('drone_node',anonymous=False)
    _cam_topic_sub = rospy.Subscriber("/video_source/raw", Image, callback)
    rospy.spin()
