## 윤혁이 포탑 마무리 ##
# 0. 모터 코드로 제어하기
# 1. opcv로 마커 따라가기
# 2. 총 발사하기

import os
import sys
import time
import math
import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from multiprocessing import Process, Queue


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 

# Control table address
ADDR_RX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_RX_GOAL_POSITION      = 30
ADDR_RX_MOVING_SPEED       = 32   
ADDR_RX_PRESENT_POSITION   = 36


# ADDR_MOVING_SPEED       = 32
# ADDR_TORQUE_LIMIT       = 34

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
LEN_MOVING_SPEED        = 4
# Protocol version
PROTOCOL_VERSION            = 1.0     

BODY_ID = 0
CAMERA_ID = 1
SHOOT_ID = 2
GUN_ID = 3

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM10'
#DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20 

#rotate_type
JOINT_TYPE = 0
WHEEL_TYPE = 1

BODY_JOINT_MIN_POS_VALUE = 204 # body id 0
BODY_JOINT_MAX_POS_VALUE = 820 

GUN_JOINT_MIN_POS_VALUE = 500 # gun id 3
GUN_JOINT_MAX_POS_VALUE = 600




def motor_control(vector_queue):
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    GOAL_POSI_Write = GroupSyncWrite(portHandler, packetHandler, ADDR_RX_GOAL_POSITION, LEN_GOAL_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    class motor():
        def __init__(self, id, rotate_type):
            self.id = id
            self.rotate_type = rotate_type
            if id == 0:
                self.max_pos_val = BODY_JOINT_MAX_POS_VALUE
                self.min_pos_val = BODY_JOINT_MIN_POS_VALUE
            elif id == 3:
                self.max_pos_val = GUN_JOINT_MAX_POS_VALUE
                self.min_pos_val = GUN_JOINT_MIN_POS_VALUE
            else:
                self.max_pos_val = 1023
                self.min_pos_val = 0

        def set_position(self, deg):
            rad = deg * math.pi/180
            value = rad * 512 / math.pi

            if value > self.max_pos_val:
                value = self.max_pos_val
            elif value < self.min_pos_val:
                value = self.min_pos_val

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.id, ADDR_RX_GOAL_POSITION, int(value))
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            
        
        def get_position(self):
            position = packetHandler.read2ByteTxRx(portHandler, self.id, ADDR_RX_PRESENT_POSITION)
            return (position[0])*360/1023

        def torque_on(self):
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.id, ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

        def shoot(self):# 발사 시작하고 1.515초 딜레이 주면 딱 1사이클 맞음(회전속도 1000 기준)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.id, ADDR_RX_MOVING_SPEED, 1000)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            time.sleep(1.515)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.id, ADDR_RX_MOVING_SPEED, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    #def cease_fire(self):
    print('\033[91m' + 'Motor control Thread On!' + '\033[0m')
     # initialize
    body_joint = motor(BODY_ID, JOINT_TYPE)
    body_joint.set_position(180)
    camera_joint = motor(CAMERA_ID, JOINT_TYPE)
    camera_joint.set_position(425*val2deg)
    gun_joint = motor(GUN_ID, JOINT_TYPE)
    gun_joint.set_position(536*val2deg)
    shoot_sw = motor(SHOOT_ID, WHEEL_TYPE)

    shoot_sw.shoot()
    body_pose_before = 0
    gun_pose_before = 0
    ttt = time.time()
    while 1:
        if vector_queue.empty() == False:
            # q_len = vector_queue.qsize()
            # while q_len > 1:
            #     t = vector_queue.get(0)
            #     q_len = q_len-1
            x, y, z = vector_queue.get(0)
            # print(vector_queue.empty())
            body_pose = body_joint.get_position()
            gun_pose = gun_joint.get_position()
            if body_pose < 360:
                body_pose_before = body_pose
            else:
                print('body over : ', body_pose%360, end=' ')
                print('body error = ', (body_pose%360) - body_pose_before)
                body_pose = body_pose_before
                
            if gun_pose < 360:
                gun_pose_before = gun_pose
            else:
                print('gun over : ', gun_pose)
                gun_pose = gun_pose_before
                
            
        
            # print(x, y, z)
            #TODO get은 현실에서의 xyz 값 출력하니까 이거 이용해서 body랑 gun 각도 atna2 이용해서 회전시키기.
            # print('now pos body : ', body_pose, end=" ")
            # print('atna2 : ', math.atan2(x,z))
            # body_joint.set_position(body_pose + math.atan2(x, z)*180/math.pi)
            # gun_joint.set_position(gun_pose + math.atan2(-y,z)*180/math.pi)
            
            body_joint.set_position(body_pose + 3*x*z/abs(x*z))
            gun_joint.set_position(gun_pose + -3*y*z/abs(y*z))
            # print(1/(time.time()-ttt))
            ttt = time.time()
        # else:
            # print('#################################empty####################33')

######################################
## 위에는 모터 제어 쓰레드 아래는 비전 ##
######################################

#Camera Intrinsic Matrix
mat_Intrin = np.array([[684.76931,   0.     , 335.16935],\
           [0.     , 689.97303, 231.6814] ,\
           [0.     ,   0.     ,   1.     ]],dtype = np.float32)
mat_distortion = np.array([-0.388776, 0.126366, -0.001096, 0.003449, 0.000000],dtype = np.float32)
rectification_matrix = np.array([[1., 0., 0.],[0., 1., 0.],[ 0., 0., 1.]])
projection_matrix = np.array([[618.70183,   0.     , 341.12391,   0.     ],
           [0.     , 655.23318, 230.47805,   0.]     ,
           [0.     ,   0.     ,   1.     ,   0.     ]])
blue_BGR = (255,0,0)
red_BGR = (0, 0, 255)
green_BGR = (0, 255, 0)


axis = np.float32([[10,0,0], [0,10,0], [0,0,-10]]).reshape(-1,3) # 마커에서 보이는 xyz축의 길이
#Video
def drawAxis(img, corners, imgpts):
   corner = tuple(corners[0].ravel())
   img = cv.line(img, corner, tuple(imgpts[0].ravel()), blue_BGR, 5)
   img = cv.line(img, corner, tuple(imgpts[1].ravel()), green_BGR, 5)
   img = cv.line(img, corner, tuple(imgpts[2].ravel()), red_BGR, 5)

   return img



def find_aruco_id(img, vector_queue, marker_type=4, total_markers=50, draw=True):
    real_marker_size = 50
    marker_3d_edges = np.array([    [0.,0.,0.],
                                    [0.,real_marker_size,0.],
                                    [real_marker_size,real_marker_size,0.],
                                    [real_marker_size,0.,0.]], dtype = np.float32).reshape((4,1,3))
######    
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_type}X{marker_type}_{total_markers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(arucoDict, arucoParam)
    corners, ids, _ = detector.detectMarkers(img)

    for corner in corners:
        corner = np.array(corner).reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corner
        # print(corner)
        topRightPoint    = (int(topRight[0]),      int(topRight[1]))
        topLeftPoint     = (int(topLeft[0]),       int(topLeft[1]))
        bottomRightPoint = (int(bottomRight[0]),   int(bottomRight[1]))
        bottomLeftPoint  = (int(bottomLeft[0]),    int(bottomLeft[1]))
        centerofmarker = (int((topLeft[0] + topRight[0]+bottomRight[0]+bottomLeft[0])/4),
                          int((topLeft[1] + topRight[1]+bottomRight[1]+bottomLeft[1])/4) )
        

        cv.circle(img, topLeftPoint, 4, blue_BGR, -1)
        cv.circle(img, topRightPoint, 4, blue_BGR, -1)
        cv.circle(img, bottomRightPoint, 4, blue_BGR, -1)
        cv.circle(img, bottomLeftPoint, 4, blue_BGR, -1)
        cv.circle(img, centerofmarker, 4, red_BGR, -1)
        
        ret, rvec, tvec = cv.solvePnP(marker_3d_edges, corner, mat_Intrin, mat_distortion)
        if ret:
            corner = np.array(corner).reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corner
            x=round(tvec[0][0]+real_marker_size/2,2) # 실제 거리 mm 에서 카메라 중심과에서 x축
            y=round(tvec[1][0]+real_marker_size/2,2)
            z=round(tvec[2][0],2)

            rmat = cv.Rodrigues(rvec)[0]

            #version2 matrix transpose
            rx = math.atan2(rmat[1][2], rmat[2][2])
            ry = math.atan2(-rmat[0][2], math.sqrt(rmat[1][2]**2+rmat[2][2]**2))
            rz = math.atan2(rmat[0][1], rmat[0][0]) - math.pi/2

            rx=round(-(((np.rad2deg(rx)+360)%360)-180),2)
            ry=round(np.rad2deg(ry),2)
            rz=round(np.rad2deg(rz),2)
            
            text1 = f"{x},{y},{z}"
            text2 = f"{rx},{ry},{rz}"
            cv.putText(img, text1, (centerofmarker[0]-40,   centerofmarker[1]+20), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
            cv.putText(img, text2, (centerofmarker[0]-40,   centerofmarker[1]+50), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1)
            
            img_shape = img.shape
            centerofImg = (int(img_shape[1]/2), int(img_shape[0]/2))
            end_point = (int(centerofmarker[0]), int(centerofmarker[1]))
            arrow_x = end_point[0] - centerofImg[0]
            arrow_y = end_point[1] - centerofImg[1]
            # if abs(arrow_x) > 5 and abs(arrow_y) > 5: # 이미지 상에서의 차이
            #    cv.arrowedLine(img, centerofImg, end_point, green_BGR, 5)
            #    vector_queue.put((end_point[0] - centerofImg[0],end_point[1] - centerofImg[1]))
            if abs(x) > 3 and abs(y) > 3: # 실제 물체의 좌표 : 이미지센서가 0,0,0
                cv.arrowedLine(img, centerofImg, end_point, green_BGR, 5)
                if vector_queue.qsize() <3:
                    vector_queue.put((x,y,z))
            
            imgpts, jac = cv.projectPoints(axis, rvec, tvec, mat_Intrin, mat_distortion)
        
            a1 = int(tuple(imgpts[0].ravel())[0])
            a2 = int(tuple(imgpts[0].ravel())[1])
            b1 = int(tuple(imgpts[1].ravel())[0])
            b2 = int(tuple(imgpts[1].ravel())[1])
            c1 = int(tuple(imgpts[2].ravel())[0])
            c2 = int(tuple(imgpts[2].ravel())[1])

            cv.line(img, topLeftPoint, (c1,c2), red_BGR, 2)
            cv.line(img, topLeftPoint, (a1,a2), blue_BGR, 2)
            cv.line(img, topLeftPoint, (b1,b2), green_BGR, 2)  

    return ids
def detect_marker_process(vector_queue):
    print('\033[91m' + 'Vision Thread On!' + '\033[0m')
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        fps = cap.get(cv.CAP_PROP_FPS)

        id = find_aruco_id(frame, vector_queue)
        #print(id)
        #cv.putText(frame, "fps : %0.1f" %fps, (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        #cv.putText(frame, "fps : %0.1f" %fps_2, (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))

        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv.imshow('frame', frame)

        # turn off
        if cv.waitKey(10) == ord('q'):
            break
        
        # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

val2deg = 180/512

if __name__=='__main__':
   

    vector_queue = Queue()
    vision_process = Process(target=detect_marker_process, args=(vector_queue,))
    motor_process = Process(target=motor_control, args=(vector_queue,))
    vision_process.start()
    motor_process.start()


    vector_queue.close()
    vision_process.join()
    motor_process.join()


# shoot_sw.shoot()



# body_joint.set_position(200)
# time.sleep(1)
# body_joint.set_position(180*math.pi/180)