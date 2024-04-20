import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import time, math

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


axis = np.float32([[10,0,0], [0,10,0], [0,0,-10]]).reshape(-1,3)
#Video
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


def drawAxis(img, corners, imgpts):
   corner = tuple(corners[0].ravel())
   img = cv.line(img, corner, tuple(imgpts[0].ravel()), blue_BGR, 5)
   img = cv.line(img, corner, tuple(imgpts[1].ravel()), green_BGR, 5)
   img = cv.line(img, corner, tuple(imgpts[2].ravel()), red_BGR, 5)

   return img



def find_aruco_id(img, marker_type=4, total_markers=50, draw=True):
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
            # print('tvec = ', tvec)
            # print('rvec = ', rvec)
            x=round(tvec[0][0]+real_marker_size/2,2)
            y=round(tvec[1][0]+real_marker_size/2,2)
            z=round(tvec[2][0],2)

            rmat = cv.Rodrigues(rvec)[0]
            

            # version1
            # rx = math.atan2(rmat[2][1], rmat[2][2])
            # ry = math.atan2(-rmat[2][0], math.sqrt(rmat[2][1]**2+rmat[2][2]**2))
            # rz = math.atan2(rmat[1][0], rmat[0][0])

            #version2 matrix transpose
            rx = math.atan2(rmat[1][2], rmat[2][2])
            ry = math.atan2(-rmat[0][2], math.sqrt(rmat[1][2]**2+rmat[2][2]**2))
            rz = math.atan2(rmat[0][1], rmat[0][0]) - math.pi/2

            #version3 well
            # sy = math.sqrt(rmat[0][0]**2 + rmat[1][0]**2)
            # singular = sy < 1e-6
            # if not singular:
            #    rx = math.atan2(rmat[2][1],rmat[2][2])
            #    ry = math.atan2(-rmat[2][0], sy)
            #    rz = math.atan2(rmat[1][0], rmat[0][0])
            # else:
            #    rx = math.atan2(-rmat[1][2], rmat[1][1])
            #    ry = math.atan2(-rmat[2][0],sy)
            #    rz = 0
               
        
            rx=round(-(((np.rad2deg(rx)+360)%360)-180),2)
            ry=round(np.rad2deg(ry),2)
            rz=round(np.rad2deg(rz),2)
            # print(np.rad2deg(rvec[0][0]))
            
            #rvec.astype(float)    
            # theta = math.sqrt((rvec[0][0])**2+(rvec[1][0])**2+(rvec[2][0])**2)
            # rx = rvec[0][0]/theta
            # ry = rvec[1][0]/theta
            # rz = rvec[2][0]/theta
            # rx=round(np.rad2deg(rvec[0][0]/theta),2)
            # ry=round(np.rad2deg(rvec[1][0]/theta),2)
            # rz=round(np.rad2deg(rvec[2][0]/theta),2)
            # print('rx, ry, rz = ', rx, ry, rz)
            # PnP 결과를 이미지에 그려 확
            text1 = f"{x},{y},{z}"
            text2 = f"{rx},{ry},{rz}"
            cv.putText(img, text1, (centerofmarker[0]-40,   centerofmarker[1]+20), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
            cv.putText(img, text2, (centerofmarker[0]-40,   centerofmarker[1]+50), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1)
            
            img_shape = img.shape
            centerofImg = (int(img_shape[1]/2), int(img_shape[0]/2))
            end_point = (int(centerofmarker[0]), int(centerofmarker[1]))
            if abs(x) > 5 or abs(y) > 5:
               cv.arrowedLine(img, centerofImg, end_point, green_BGR, 5)
            
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
            

            # print('Blue axis(y) = ',a1-topLeftPoint[0] , a2-topLeftPoint[1])
            # print('Green axis(x) = ',b1-topLeftPoint[0] , b2-topLeftPoint[1])
            # print('Red axis(z) = ',c1-topLeftPoint[0] , c2-topLeftPoint[1])
        

    return ids



while True:
 # Capture frame-by-frame
 ret, frame = cap.read()
 
 # if frame is read correctly ret is True
 if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break

 fps = cap.get(cv.CAP_PROP_FPS)

 id = find_aruco_id(frame)
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