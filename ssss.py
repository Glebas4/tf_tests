import numpy as np
import rospy
from std_msgs.msg import String
from clover import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import cv2 as cv
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()
#set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


colors = {
    'yellow' : np.array([[55, 240, 240], [65, 255, 255]]),
    'red'    : np.array([[111, 111, 111], [111, 111, 111]]),
    'green'  : np.array([[111, 111, 111], [111, 111, 111]]),
    'blue'   : np.array([[111, 111, 111], [111, 111, 111]])
}




def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)



def detect_color(img) -> str:
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    for key, color in color.items():
        if cv.countNonZero(cv.inRange(hsv, color[0], color[1])) > 10:
            return key
        
    return ''



def detect_qr(img) -> str:
    barcodes = pyzbar.decode(img)
    if len(barcodes):
        return barcodes[0].data.decode('utf-8')
    
    return ''



def scan():
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    color = detect_color(img)
    qr = detect_qr(img)

    return [color, qr]
    

def main():
    x = 0
    y = 0
    vpravo = True
    i = 0
    navigate_wait(z=1.5, frame_id='body', auto_arm=True)

    #while y < 2.5:
    #    if vpravo:
    #        while i != 8:
    #            navigate_wait(x, y, 1.5, frame_id='aruco_map')
    #            print(scan())
    #            x += 0.5
    #            i += 1

    #    else:
    #        while i != 8:
    #            navigate_wait(x, y, 1.5, frame_id='aruco_map')
    #            print(scan())
    #            x -= 0.5
    #            i += 1

    #    navigate_wait()
    #    print(scan)
    #    y += 0.5
    #    i = 0

    #    vpravo = not vpravo

    navigate_wait(x=2, y=2, z=1.5, frame_id='aruco_map')

    #navigate_wait(x=4, y=1, z=1.5, frame_id='aruco_map')

    navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_map')
    land()
        

        
if __name__ == '__main__':
    main()


