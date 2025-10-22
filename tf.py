import cv2 as cv
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from clover import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
#from clover import long_callback
from sensor_msgs.msg import CameraInfo
import numpy as np



get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()
#pub = rospy.Publisher('buildings', String, queue_size=1)
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)
tf_buffer.can_transform('aruco_map', 'camera_link', rospy.Time(0), rospy.Duration(5.0))
K = D = Z = 0
buildings = []
colors = {
    "red"   : ((0, 0, 220),(50, 50, 255)),
    "green" : ((0, 220, 0),(50, 255, 50)),
    "blue"  : ((255, 0, 0),(255, 60, 60)),
    "yellow": ((0, 220, 220),(0, 255, 255)) 
}



def range_callback(msg):
    global Z
    Z = msg.range


def cam_info():
    global K, D
    msg = rospy.wait_for_message('/main_camera/camera_info', CameraInfo, timeout=1) # Информация о камере
    K = np.array(msg.K).reshape(3, 3) #Создание матрицы 3x3
    D = np.array(msg.D) #Коэффициенты дисторсии
    return K, D


def scan(img):
    detects = []
    for col, val in colors.items():
        bin = cv.inRange(img, val[0], val[1])
        count = cv.countNonZero(bin)
        if count > 30:
            detects.append((col, bin))
        
    return detects


def get_centers(bin_img):
    contours, _ = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cords = []
    for cnt in contours:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            cords = np.array([[[x, y]]], dtype=np.float32)
    return cords
        

def transform(final_point, P_cam):
    final_point.header.frame_id = 'main_camera_optical'
    final_point.point.x = P_cam[0]
    final_point.point.y = P_cam[1]
    final_point.point.z = P_cam[2]
    final_point = tf_buffer.transform(final_point, 'aruco_map', rospy.Duration(1.0))
    return final_point


#@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    detects = scan(img)
    if detects:
        for pnt in detects:
            cords = get_centers(pnt[1])
            cords_norm = cv.undistortPoints(cords, K, D, P=None)
            x = float(cords_norm[0,0,0])
            y = float(cords_norm[0,0,1])
            r_cam = np.array([x, y, 1.0])
            P_cam = r_cam * Z # Точка в координатах камеры
            final_point = geometry_msgs.msg.PointStamped()
            result = transform(final_point, P_cam)
            print(result)
            


def main():
    if Z < 1:
        navigate(x=0, y=0, z=2.5, frame_id="body", auto_arm=True)
        rospy.sleep(5)

   
if __name__ == '__main__':
    rospy.init_node('flight')
    rospy.Subscriber('rangefinder/range', Range, range_callback)
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    K, D = cam_info()
    main()
    rospy.spin()
