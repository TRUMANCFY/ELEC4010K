#!/usr/bin/env python
# Python Image Subscriber Node
# This node was modified with reference to imcmahon's reply on
# http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
# Basic idea is convert from ROS Image -> CvBridge Converter -> OpenCV
# Note you'd still need the CMakeList.txt and package.xml
# Reference:
#  http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
#  http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import rospy                      # rospy
import numpy as np                # numpy
import cv2                        # OpenCV2
import math
from sensor_msgs.msg import Image, LaserScan # ROS Image message
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter

from geometry_msgs.msg import Pose,PoseStamped, Point, Quaternion, Vector3, Polygon
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#Instantiate CV Bridge
bridge = CvBridge()
pattern_list = {}
hist_list = {}
pic_list = []
marker_array = MarkerArray()
dis=0
slam_pose = PoseStamped()
name_list = ['obama', 'avril', 'zhang', 'prince', '2D']
not_find_name_list = ['obama', 'avril', 'zhang', 'prince', '2D']
color_list = [[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0], [1.0,1.0,0.0], [0.0,1.0,1.0]]
laser_reading = LaserScan()

def get_slam_pose(slam_out_pose):
    global slam_pose
    slam_pose = slam_out_pose

def get_laser_scan(laser_scan):
    global laser_reading
    laser_reading = laser_scan

def Marker_place(pos, dis,color):
    global marker_array
    imageMarker = Marker()
    imageMarker.header.frame_id = "/map"
    imageMarker.ns = "image"
    imageMarker.id = color_list.index(color)
    imageMarker.type = 2 # sphere
    imageMarker.action = 0
    t1 = 2.0 * (pos.pose.orientation.w * pos.pose.orientation.z + pos.pose.orientation.x * pos.pose.orientation.y)
    t2 = 1.0 - 2.0 * (pos.pose.orientation.y * pos.pose.orientation.y + pos.pose.orientation.z * pos.pose.orientation.z)
    z = math.degrees(math.atan2(t1,t2)) - 90
    h = len(laser_reading.ranges)/2
    imageMarker.pose.position.x = pos.pose.position.x + laser_reading.ranges[h]*np.cos(z*(2*np.pi)/360)
    imageMarker.pose.position.y = pos.pose.position.y + laser_reading.ranges[h]*np.sin(z*(2*np.pi)/360)
    imageMarker.pose.position.z = pos.pose.position.z
    imageMarker.pose.orientation.x = 0
    imageMarker.pose.orientation.y = 0
    imageMarker.pose.orientation.z = 0
    imageMarker.pose.orientation.w = 1
    imageMarker.scale.x = 0.3
    imageMarker.scale.y = 0.3
    imageMarker.scale.z = 0.3
    imageMarker.color.r = color[0]
    imageMarker.color.g = color[1]
    imageMarker.color.b = color[2]
    imageMarker.color.a = 1.0
    imageMarker.lifetime = rospy.Duration()
    marker_array.markers.append(imageMarker)

def init_pattern():
    print("initialize pattern")
    global pattern_list
    global hssssssist_list
    img1 = cv2.imread("/home/jliubd/catkin_ws/src/ROS_py_image_subscriber/script/pic001.jpg", 1)
    img2 = cv2.imread("/home/jliubd/catkin_ws/src/ROS_py_image_subscriber/script/pic002.jpg", 1)
    img3 = cv2.imread("/home/jliubd/catkin_ws/src/ROS_py_image_subscriber/script/pic003.jpg", 1)
    img4 = cv2.imread("/home/jliubd/catkin_ws/src/ROS_py_image_subscriber/script/pic004.jpg", 1)
    img5 = cv2.imread("/home/jliubd/catkin_ws/src/ROS_py_image_subscriber/script/pic005.jpg", 1)
    s = 15

    img_list = []
    img_list1 = []
    img1_ = cv2.resize(img1, (s, s))
    img2_ = cv2.resize(img2, (s, s))
    img3_ = cv2.resize(img3, (s, s))
    img4_ = cv2.resize(img4, (s, s))
    img5_ = cv2.resize(img5, (s, s))


    img_list.append(img1)
    img_list.append(img2)
    img_list.append(img3)
    img_list.append(img4)
    img_list.append(img5)

    img_list1.append(img1_)
    img_list1.append(img2_)
    img_list1.append(img3_)
    img_list1.append(img4_)
    img_list1.append(img5_)
    global name_list
    #name_list = ['obama', 'avril', 'zhang', 'prince', '2D']


    for i in range(5):
        hist = []
        for j in range(3):
            hist1 = cv2.calcHist([img_list[i]], [j], None, [256], [0, 255])
            hist.append(hist1)
        hist_list[name_list[i]] = hist


    for i in range(5):
        pattern_list[name_list[i]] = img_list1[i]

def comparehist(img):
    sim = []
    max_name = None
    max_value = 0.0
    value_dict = {}
    value_list = []
    for name, histref in hist_list.items():
        sim_each = []
        for i in range(3):
            hist1 = cv2.calcHist([img], [i], None, [256], [0, 255])
            sim1 = cv2.compareHist(hist1, histref[i], cv2.HISTCMP_CORREL)
            sim_each.append(sim1)
        print(np.mean(sim_each))
        value = np.mean(sim_each)
        value_dict[name] = value
        value_list.append(value)
        if np.mean(sim_each) > max_value:
            max_name = name
            max_value = np.mean(sim_each)
    value_list_sort = np.sort(value_list)
    second_sim = value_list_sort[-2]
    second_name = list(value_dict.keys())[list(value_dict.values()).index(second_sim)]
    if max_value > 0.3:
        return max_name
    else:
        return None

def comparepattern(img):
    s = 15
    img_ = cv2.resize(img, (s, s))
    min_name = None
    min_value = 0.0
    value_dict = {}
    value_list = []
    for name, pattern in pattern_list.items():
        value = np.sum(np.abs(pattern - img_))
        print(value)
        value_dict[name] = value
        value_list.append(value)

        if min_name == None:
            min_name = name
            min_value = value

        if value < min_value:
            min_name = name
            min_value = value

    value_sort = np.sort(value_list)
    second_min = value_sort[1]
    second_name = list(value_dict.keys())[list(value_dict.values()).index(second_min)]
    if second_min - min_value > min_value * 0.1:
        print(min_name)
        return True, min_name
    else:
        a = comparehist(img)
        print(a)
        #print(b)
        print(min_name)
        print(second_name)
        if a == min_name or a == second_name:
            print(a)
            return True, a

        else:
            return False, None
    return False, None

def segment(img, pos):
    '''
    left, right = np.min(box[:, 0]), np.max(box[:, 0])
    bottom, top = np.min(box[:, 1]), np.max(box[:, 1])
    imgnew = img[left:right, bottom:right, :]
    '''
    imgnew = img[pos[1]:pos[1]+pos[3], pos[0]:pos[0]+pos[2], :]
    return imgnew

def check(img, img_org):
    if img.shape[0] < 0.2 * img_org.shape[0] or img.shape[1] < 0.2 * img_org.shape[1]:
        return False
    if img.shape[0] > 0.8 * img_org.shape[0] or img.shape[1] > 0.8 * img_org.shape[1]:
        return False
    print((1.0 * img.shape[0]) / img.shape[1])
    if (1.0 * img.shape[0]) / img.shape[1] > 1.1 or (1.0 * img.shape[0]) / img.shape[1] < 0.9:
        return False

    for i in range(3):
        print(np.unique(img[:, :, i]).size)
        if np.unique(img[:, :, i]).size < 50:
            return  False

    return True

#def findpattern(img_seg, pattern_list):
def img_extract(cv2_img):
    global dis
    global not_find_name_list
    global color_list
    cv2_img = cv2.flip(cv2_img, 1)
    lap = cv2.Laplacian(cv2_img, cv2.CV_8U)
    kernel = np.ones((5, 5), np.uint8)

    #lap = cv2.erode(lap.astype(np.float32), kernel1, iterations = 1)
    img_plus = (lap>75) * 255

    img_plus = cv2.dilate(img_plus.astype(np.float32), kernel, iterations = 15)
    img_plus = cv2.erode(img_plus.astype(np.float32), kernel, iterations = 20)
    img_plus = cv2.dilate(img_plus.astype(np.float32), kernel, iterations = 5)




    gray = cv2.cvtColor(img_plus, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, 1)
    thresh = thresh.astype(np.uint8)
    img2, contours, h = cv2.findContours(thresh, cv2.RETR_TREE , cv2.CHAIN_APPROX_SIMPLE)
    img_new = cv2_img
    if len(contours) == 1:
        print("not catch")
        return lap, img2, img_plus, cv2_img, cv2_img
    else:
        print(len(contours))
        for i in range(1, len(contours)):
            cnt = contours[i]
            '''
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            '''
            pos = cv2.boundingRect(cnt)
            img_seg = segment(cv2_img, pos)
            x, y, w, h = pos
            b, n = comparepattern(img_seg)
            print b,n
            if check(img_seg, cv2_img) and b:
                '''
                img2 = cv2.drawContours(img2, [box], 0, (255, 0, 0), 2)
                img_new = cv2.drawContours(img_new, [box], 0, (255, 0, 0), 2)
                '''
                img2 = cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 0, 255), 2)
                img_new = cv2.rectangle(img_new, (x, y), (x+w, y+h), (0, 0, 255), 2)
                img_plus = cv2.rectangle(img_plus, (x, y), (x+w, y+h), (0, 0, 255), 2)
                if n in not_find_name_list:
                    dis = 256/(0.4142*h)
                    color_index = name_list.index(n)
                    color = color_list[color_index]
                    Marker_place(slam_pose,dis,color)
                    not_find_name_list.remove(n)
                    print(not_find_name_list)
            else:
                '''
                img2 = cv2.drawContours(img2, [box], 0, (127, 127, 127), 2)
                img_new = cv2.drawContours(img_new, [box], 0, (127, 127, 127), 2)
                '''
                img2 = cv2.rectangle(img2, (x, y), (x+w, y+h), (127, 127, 127), 2)
                img_new = cv2.rectangle(img_new, (x, y), (x+w, y+h), (127, 127, 127), 2)
                img_plus = cv2.rectangle(img_plus, (x, y), (x+w, y+h), (127, 127, 127), 2)

            print("finish catching")
        return lap, img2,img_plus,img_new, img_seg


def image_callback(msg):
    print("PyImageSubscriber node  Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        lap, img_track,img_plus,img, img_get= img_extract(cv2_img)
    except CvBridgeError, e:
        print(e)
    else:
        # Display the converted image
        marker_array_Pub = rospy.Publisher('marker_array', MarkerArray, queue_size=100)
        marker_array_Pub.publish(marker_array)
        rospy.Subscriber("/slam_out_pose", PoseStamped, get_slam_pose)
        rospy.Subscriber("/vrep/scan", LaserScan, get_laser_scan)
        cv2.imshow("lap", lap)
        cv2.imshow("Image Display", img_track)
        #cv2.imshow("Laplacian", lap)
        cv2.imshow("img_plus", img_plus)
        cv2.imshow("original", img)
        cv2.imshow("get", img_get)
        # Wait 30 ms to allow image to be drawnself.
        # Image won't display properly without this cv2.waitkey
        cv2.waitKey(30)
        # Save your OpenCV2 image as a jpeg
        # cv2.imwrite('camera_image.jpeg', cv2_img)


def image_listener():
    # Initiate the node
    rospy.init_node('py_image_listener')
    # Setupt the subscription, camera/rb/image_raw is used in turtlebot_gazebo example
    rospy.Subscriber("vrep/image", Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.destroyWindow("Image Display")




if __name__ == '__main__':
    init_pattern()
    image_listener()
