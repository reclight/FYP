#!/usr/bin/env python
import numpy as np
import cv2
import glob
from matplotlib import pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import tf
import math
from cv_bridge import CvBridge, CvBridgeError
import roslaunch
import sys
from std_srvs.srv import Empty
from std_srvs.srv import SetBool

#frame = Image()

#def callback(data):
#    global frame
#    print(data)
#    frame = data

class SiftHomography:
    def __init__(self):
        rospy.init_node('siftpublisher', anonymous=True)
        self.bridge = CvBridge()
        self.cv_image = None
        self.CamInfo = CameraInfo()
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1, latch=True)
        self.pose_status_pub = rospy.Publisher('poseStatus', Bool, queue_size=1, latch=True)
        self.gripper_size_pub = rospy.Publisher('GripperPosition', Int64, queue_size=1, latch=True)
        self.object_height_pub = rospy.Publisher('ObjectHeight', Float32, queue_size=1, latch=True)
        self.image_pub = rospy.Publisher('ImagePosition', Image)
        self.rate = rospy.Rate(1)
        self.curr_stamp = rospy.Time.now()
        self.counter=0
        self.x_pos = np.zeros(11,dtype=float)
        self.y_pos = np.zeros(11,dtype=float)
        self.z_pos = np.zeros(11,dtype=float)
        self.x_rot = np.zeros(11,dtype=float)
        self.y_rot = np.zeros(11,dtype=float)
        self.z_rot = np.zeros(11,dtype=float)
        self.id = '2'
        self.offset = 0


    def Imagecallback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)


        poseMsg = PoseStamped()
        MIN_MATCH_COUNT = 20

        mtx = np.array([self.CamInfo.P[0],self.CamInfo.P[1],self.CamInfo.P[2],self.CamInfo.P[4],self.CamInfo.P[5],self.CamInfo.P[6],self.CamInfo.P[8],self.CamInfo.P[9],self.CamInfo.P[10]])
        mtx=mtx.reshape(3,3)

        dist = self.CamInfo.D
        
        #product choices
        self.id = rospy.get_param("/object", default='3')
        
        
        if self.id == '1':
            #koko
            img0 = cv2.imread('/home/reclight/ur_sift_ws/src/myworkcell_core/script/kokoside.jpg')  
            size_x = 0.108
            size_y = 0.035
            size_h = 0.1
            self.offset = 5
        elif self.id == '2':
            #milo
            img0 = cv2.imread('/home/reclight/ur_sift_ws/src/myworkcell_core/script/milocropped.jpg')
            size_x = 0.119
            size_y = 0.048
            size_h = 0.06
            self.offset = 20
        elif self.id == '3':
            #bonjela
            img0 = cv2.imread('/home/reclight/ur_sift_ws/src/myworkcell_core/script/cocoaballs.jpg')
            size_x = 0.112
            size_y = 0.030
            size_h = 0.04
            self.offset = 20

        img1 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        axis = np.float32([[0.02,0,0], [0,0.02,0], [0,0,-0.02]]).reshape(-1,3)

        frame = self.cv_image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #create SIFT keypoints
        sift = cv2.xfeatures2d.SIFT_create()

        kp1, des1 = sift.detectAndCompute(img1, None)
        kp2, des2 = sift.detectAndCompute(gray, None)

        #Feature matching with FLANN
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        good = []

        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        siftkey = cv2.drawKeypoints(img1, kp1, img0, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if len(good)>MIN_MATCH_COUNT:
            status = Bool()
            status.data=True
            self.pose_status_pub.publish(status)
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()

            h,w = img1.shape

            

            objectheight = Float32()
            objectheight.data = size_h


            grippersize = int(255 - size_y * 3000 + self.offset)
            self.gripper_size_pub.publish(grippersize)
            self.object_height_pub.publish(objectheight)

            #obj for pose estimation
            objp = np.float32([ 
                [-size_x/2, -size_y/2,0],
                [-size_x/2, size_y/2,0], 
                [size_x/2, size_y/2,0], 
                [size_x/2, -size_y/2,0] ])

            #objp = np.float32([ 
            #    [0, 0,0],
            #    [0, size_y,0], 
            #    [size_x, size_y,0], 
            #    [size_x, 0,0] ])

            #pts for sift homography
            pts = np.float32([ [0,0],[0,h-1], [w-1, h-1], [w-1, 0], [(w-1)/2, (h-1)/2] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts, M)


            frame = cv2.polylines (frame, [np.int32(dst[:4])], True, 255, 3, cv2.LINE_AA)
            
            
            _,rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, dst[:4], mtx, dist, useExtrinsicGuess=True,iterationsCount=100, flags=cv2.SOLVEPNP_UPNP)
            #print("rvecs : ", rvecs)
            #print("tvecs : ", tvecs)

            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            
            draw_params = dict(matchColor = (0, 255,0), singlePointColor=None, matchesMask=matchesMask,flags = 2)
            #pose = self.draw(frame,dst,imgpts)
            img3 = cv2.drawMatches(img1, kp1, gray, kp2, good, None, **draw_params)
            
            #cv2.imshow("Pose", pose)
            
            #cv2.imshow("Homography", gray)
            

            rot,_ = cv2.Rodrigues(rvecs)
            rotate_to_ros=np.matrix([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
            rot = np.matmul(rot, rotate_to_ros)

            rot_r, rot_p, rot_y=self.rotationMatrixToEulerAngles(rot)

            self.x_pos[self.counter]=tvecs[0]
            self.y_pos[self.counter]=tvecs[1]
            self.z_pos[self.counter]=tvecs[2]

            self.x_rot[self.counter]=rot_r
            self.y_rot[self.counter]=rot_p
            self.z_rot[self.counter]=rot_y

            if self.counter>=9:
                tvecs_x = np.median(self.x_pos)
                tvecs_y = np.median(self.y_pos)
                tvecs_z = np.median(self.z_pos)
                rvecs_r = np.median(self.x_rot)
                rvecs_p = np.median(self.y_rot)
                rvecs_y = np.median(self.z_rot)
                poseMsg.header.stamp=self.CamInfo.header.stamp
                poseMsg.header.frame_id="camera_frame"

                poseMsg.pose.position.x = tvecs_x
                poseMsg.pose.position.y = tvecs_y
                poseMsg.pose.position.z = tvecs_z

                q = tf.transformations.quaternion_from_euler(rvecs_r, rvecs_p, rvecs_y)
                
                poseMsg.pose.orientation.x = q[0]
                poseMsg.pose.orientation.y = q[1]
                poseMsg.pose.orientation.z = q[2]
                poseMsg.pose.orientation.w = q[3]

                self.pose_pub.publish(poseMsg)

                self.counter = 0
                
                
                

            self.counter =self.counter+1
        else:
            print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
            status = Bool()
            status.data = False
            self.pose_status_pub.publish(status)
            matchesMask = None

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)
        

    def CamInfoCallback(self,data):
        self.CamInfo=data
    
    
    #Callback 
    def detectImage(self):
        camera_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.CamInfoCallback)
        image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.Imagecallback)
        rospy.spin()


    #draw function
    def draw(self, img, corners, imgpts):
        corner = tuple(corners[4].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R) :
    
        assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])


if __name__ == "__main__":
    print(sys.version)
    sifthomo = SiftHomography()
    sifthomo.detectImage()
    rospy.spin()
    

        
