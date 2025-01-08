import os, sys, time, datetime
import ydlidar
import serial
import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import cv2

import pickle

def STOP_ALL():
    while(1):
        continue

#Aruco Marker Initialization------------------------------------------------#
print('***Aruco Marker Initialization***')

def create_files():
    num = 50 # cv2.aruco.DICT_4X4_50
    size = 200 # pixel (w,h)

    for i in range(num):
        markerImage = np.zeros((size, size), dtype=np.uint8)
        markerImage = cv2.aruco.drawMarker(markerDictionary, i, size, markerImage, 1)
        if not os.path.exists("./data"):
            os.mkdir('./data')
        cv2.imwrite("./data/data{}.jpg".format(i), markerImage)

tracMarker = [25]
markerSize = 18.0
print(f'-Tracking Marker No. : {tracMarker}')
print(f'-Size of Marker : {markerSize}cm')

print('Create Marker', end='')
markerDictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50); print('.', end='')
arucoParameter = cv2.aruco.DetectorParameters_create(); print('.', end='')
create_files(); print('.', end='')
print('Done')

#Serial Communication Initialization------------------------------------------------#
print('***Serial Communication Initialization***', end='')

portSerial = '/dev/ttyUSB0'
try:
    ser = serial.Serial(portSerial, 9600); print('.', end='')    
except:
    print(f'-Serial Port [{portSerial}]is not available')
    STOP_ALL()
finally:
    print('.', end='')
    
time.sleep(2); print('.', end='')
print('Done')

#GPIO Initialization------------------------------------------------#
print('***GPIO Initialization***')
led_onCamera = 14
led_onLidar = 15
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM); print('.', end='')
GPIO.setup(led_onCamera, GPIO.OUT)
GPIO.setup(led_onLidar, GPIO.OUT); print('.', end='')
GPIO.output(led_onCamera, GPIO.LOW)
GPIO.output(led_onLidar, GPIO.LOW); print('.', end='')
print('Done')

#Camera Initialization------------------------------------------------#
print('***Camera Initialization***')
try:
    cap = cv2.VideoCapture(0)
except:
    print('\nCamera is not available')
    STOP_ALL()
finally:
    if cap.isOpened():
        print('Camera is available')
    else:
        print('Camera is not available')
print('*Camera default setting : ')
cap.set(cv2.CAP_PROP_FPS, 20)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)


cameraBuffer = cap.get(cv2.CAP_PROP_BUFFERSIZE)
print(f'-Buffer size : {cameraBuffer}')
cameraFPS = cap.get(cv2.CAP_PROP_FPS)
cameraWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cameraHeight= cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
font = cv2.FONT_HERSHEY_SIMPLEX
fontSize = 0.8
print(f'-FPS : {cameraFPS}')
print(f'-Resolution : {cameraWidth}x{cameraHeight} pixel')

print('*Camera calibration setting')
path_calib = 'Camera_Calibration_Data'
file_cameraMatrix = 'cameraMatrix_origin.txt'
file_cameraDistortion = 'cameraDistortion_origin.txt'
cameraMatrix = np.loadtxt(os.path.join(path_calib, file_cameraMatrix), delimiter=',')
cameraDistortion = np.loadtxt(os.path.join(path_calib, file_cameraDistortion), delimiter=',')
print(f'-Calibration data directory : {path_calib}')
print(f'Camera Matrix : {os.path.join(path_calib, file_cameraMatrix)}')
print(cameraMatrix)
print(f'Camera Distortion : {os.path.join(path_calib, file_cameraMatrix)}')
print(cameraDistortion)
print('Done')

#Lidar Initialization------------------------------------------------#
print('***LiDAR Initialization***')

class cnf(object):
    boundary = {
        'minX':-3.0,
        'maxX': 3.0,
        'minY':-3.0,
        'maxY': 3.0
    }
    human = {
        'width_m':0.7,
        'length_m':0.7
        }
    BEV_WIDTH = 240
    BEV_HEIGHT = 240
    DISCRETIZATION = (boundary['maxX'] - boundary['minX'])/BEV_HEIGHT
    RADIUS = 0.1/DISCRETIZATION
    CENTER = np.array([BEV_WIDTH/2, BEV_HEIGHT/2])   
    human['width_pix'] = human['width_m']/DISCRETIZATION
    human['length_pix'] = human['length_m']/DISCRETIZATION
    
    
def extractPointsInArea(pointCloud_, boundaryCond):
    xMask = (boundaryCond['minX']<=pointCloud_[:,0]) & (pointCloud_[:,0]<=boundaryCond['maxX'])
    yMask = (boundaryCond['minY']<=pointCloud_[:,1]) & (pointCloud_[:,1]<=boundaryCond['maxY'])
    mask = np.where(xMask & yMask)
    
    pointCloud = pointCloud_[mask]
    return pointCloud

def makeBEVimage(pointCloud_, boundaryCond, discretization):
    height = cnf.BEV_HEIGHT
    width = cnf.BEV_WIDTH
    
    pointCloud = pointCloud_.copy()
    
    pointCloud[:,0] = np.int_(np.floor(pointCloud_[:,0]/discretization)+height/2)
    pointCloud[:,1] = np.int_(np.floor(pointCloud_[:,1]/discretization)+width/2)
    
    bev = np.zeros((height, width))
    bev[np.int_(pointCloud[:,0]), np.int_(pointCloud[:,1])] = pointCloud[:,2]/1023
    bev = np.flip(bev, axis=0)
    return np.array(255*bev, dtype=np.uint8) 

def polar2cart(r, deg):
    x = r*np.cos(deg)
    y = r*np.sin(deg)
    data = np.array([x, y]).T
    return data

print('**BEV setting')
print(f'-Shape of BEV image : {cnf.BEV_HEIGHT}x{cnf.BEV_WIDTH}')
print(f'-1 Pixel of BEV image : {cnf.DISCRETIZATION:.4f} m')
print(f'-Shape of human : (width, length) = {(cnf.human["width_m"], cnf.human["length_m"])}m = {(cnf.human["width_pix"], cnf.human["length_pix"])}pixel')

OPENCV_OBJECT_TRACKERS = OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create,
        "goturn": cv2.TrackerGOTURN_create
    }

trackerInitializer = OPENCV_OBJECT_TRACKERS['csrt']

trackerName = str(trackerInitializer())[1:].split(' ')[0]
tracker = None
print(f'-Tracker type : {trackerName}')

for _, value in ydlidar.lidarPortList().items():
    portLidar = value
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, portLidar);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
scan = ydlidar.LaserScan()

isLidarAv, isLaserAvailalbe, lidarScan = None, None, None

isLidarAv = laser.initialize();
if isLidarAv:
    print('Turn on the laser', end='')
    isLaserAv = laser.turnOn();
    if isLaserAv:
        print('...Done')
        lidarScan = ydlidar.LaserScan()
    else:
        print('Laser is not available')
        laser.turnOff()
        laser.disconnecting()
        STOP_ALL()
else:
    print('Lidar is not available')
    laser.turnOff()
    laser.disconnecting()
    STOP_ALL()
        
#Notify the start------------------------------------------------#
for count in range(6):
    GPIO.output(led_onCamera, count%2)
    GPIO.output(led_onLidar, count%2)
    time.sleep(0.2)
GPIO.output(led_onCamera, GPIO.LOW)
GPIO.output(led_onLidar, GPIO.LOW)


#Interface Setting---------------------------------------------#
class args(object):
    showResult = True
    saveTimeInfo = False
    
    saveVideo = False
    videoPath = './videos'
    videoName = f'MainVehicleCode_{datetime.datetime.now().strftime("%Y%m%d_%H_%M_%S")}.avi'
    
#Global Variables---------------------------------------------#
class Timer(args):
    def __init__(self, flagName):
        if not args.saveTimeInfo:
            return
        self.flagName = flagName
        self.cur = time.time()
        
    def Done(self, timeDict):
        if not args.saveTimeInfo:
            return None
        timeDict.update({self.flagName: time.time()-self.cur})

class State:
    def __init__(self):
        self.cur = 0
        self.pre = 0
        
def getEulerAngle(R):
    cy = np.sqrt(R[0,0]**2+R[1,0]**2)
    if cy>1e-6:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], cy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], cy)
        z = 0
    return np.array([x, y, z])

def pix2coor(pixRow, pixCol, imgSize, discretization):
    width, height = imgSize
    
    posX = (height/2-pixRow)*discretization
    posY = (pixCol-width/2)*discretization
    
    return posX, posY

timer = State()
Rflip = np.zeros((3,3), dtype=np.float32); Rflip[0,0] = 1.0; Rflip[1,1] = -1.0; Rflip[2,2] = -1.0
isTracMarkerEx = State()
initBbox = None
isBboxDeclared = False
posInfo = {'Found': 0, 'Dist':0, 'Angle':0}

#Main Process------------------------------------------------#
timeArray = []
frameArray = []
time_std = 0

while(cap.isOpened() and ydlidar.os_isOk()):
    timeDict = {}
    
    tcheck = Timer('A Get video from Camera')
    isCameraAv, imgCamera = cap.read()
    imgCamera = cv2.rotate(imgCamera, cv2.ROTATE_180)
    tcheck.Done(timeDict)
        
    isTracMarkerEx.cur = False
    tvec, rvec = None, None
    
    
    tcheck = Timer('B Get point cloud from LiDAR')
    isLidarAv = laser.doProcessSimple(scan)
    tcheck.Done(timeDict)
    
    if isCameraAv:
        tcheck = Timer('C Check marker from video')
        markerVertexInfoSet, markerIDset, _ = cv2.aruco.detectMarkers(imgCamera, markerDictionary, parameters=arucoParameter, cameraMatrix=cameraMatrix, distCoeff=cameraDistortion)       

        isTracMarkerEx.cur = tracMarker in markerIDset if markerIDset is not None else False
        tcheck.Done(timeDict)
        
        if isTracMarkerEx.cur:
            idxTracMarker = list(markerIDset).index(tracMarker)
            
            tcheck = Timer('D Get marker pose from video')
            markerPositionInfo = cv2.aruco.estimatePoseSingleMarkers(markerVertexInfoSet[idxTracMarker], markerSize, cameraMatrix, cameraDistortion)
            rvec, tvec = markerPositionInfo[0][0,0,:], markerPositionInfo[1][0,0,:]
            markerAttitude = getEulerAngle(np.matmul(Rflip, np.matrix(cv2.Rodrigues(rvec)[0]).T))
            tcheck.Done(timeDict)
            #For Human----------------------------------------#
            if args.showResult or args.saveVideo:
                tcheck = Timer('E Video human marker interface')
                cv2.aruco.drawAxis(imgCamera, cameraMatrix, cameraDistortion, rvec, tvec, 10)
                
                markerVertex = np.squeeze(markerVertexInfoSet[idxTracMarker])
                markerCenter = np.sum(markerVertex, axis=0)/4
                markerAttitude_ = np.rad2deg(markerAttitude)
                cv2.circle(imgCamera, tuple(map(int, markerCenter)), 2, (255, 255, 255), -1)
                for i in range(4):
                        cv2.line(imgCamera, tuple(markerVertex[i,:]), tuple(markerVertex[(i+1)%4,:]), (180, 220, 255), 1)
                msgPosition = f'Marker Position : x={tvec[0]:.2f} y={tvec[1]:.2f} z={tvec[2]:.2f}'
                msgAttitude = f'Marker Attitude : r={markerAttitude_[0]:.2f} p={markerAttitude_[1]:.2f} y={markerAttitude_[2]:.2f}'
                cv2.putText(imgCamera, msgPosition, (10, int(cameraHeight-60)), font, fontSize, (0, 255, 140), 2, cv2.LINE_AA)
                cv2.putText(imgCamera, msgAttitude, (10, int(cameraHeight-30)), font, fontSize, (0, 255, 140), 2, cv2.LINE_AA)
                
                tcheck.Done(timeDict)
            #-------------------------------------------------#
        
        #For Human----------------------------------------#
        if args.showResult or args.saveVideo:
            tcheck = Timer('F Video human coordinate interface')
            cv2.line(imgCamera, (0, int(cameraHeight/2)), (int(cameraWidth-1), int(cameraHeight/2)), (200, 200, 200), 1)
            cv2.line(imgCamera, (int(cameraWidth/2), 0), (int(cameraWidth/2), int(cameraHeight-1)), (200, 200, 200), 1)
            cv2.rectangle(imgCamera, (0, 0), (int(cameraWidth-1), int(cameraHeight-1)), (200, 200, 200), 2)
            tcheck.Done(timeDict)
        #-------------------------------------------------#
        
        
    if isLidarAv:
        tcheck = Timer('G Get BEV from pointcloud')
        data = np.array([[point.range, point.angle, point.intensity] for point in scan.points])
        lidarData = np.copy(data)
        lidarData[:,:2] = polar2cart(lidarData[:,0], lidarData[:,1])

        pointCloud = extractPointsInArea(lidarData, cnf.boundary)
        imgBEV = makeBEVimage(pointCloud, cnf.boundary, cnf.DISCRETIZATION)
        
        tcheck.Done(timeDict)
        
        posInfo['Found'] = 0
        
        if isTracMarkerEx.cur:
            tcheck = Timer('H Check marker position on BEV')
            markerPosX, markerPosY = tvec[2]+10.0, -tvec[0]
            markerPixX, markerPixY = np.int_(cnf.CENTER+np.array([-markerPosY, -markerPosX])/(100*cnf.DISCRETIZATION))
            
            initBbox_ = tuple(map(int, [markerPixX-cnf.human['width_pix']/2,
                                        markerPixY-cnf.human['length_pix']/2,
                                        cnf.human['width_pix'],
                                        cnf.human['length_pix']]))
            
            tcheck.Done(timeDict)
            #For Human----------------------------------------#
            xBbox, yBbox, BboxWidth, BboxHeight = initBbox_           

            imgBbox = imgBEV[yBbox:yBbox+BboxHeight, xBbox:xBbox+BboxWidth]
            _, binImgBbox = cv2.threshold(imgBbox, 127, 255, 0)
            contours, _ = cv2.findContours(binImgBbox, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            
            ankleList = []
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                ankX, ankY = float(x+w/2), float(y+h/2) 
                
                if ankleList is None:
                    pass
                else:
                    isOverlap = False
                    for (ankX_, ankY_, _) in ankleList:
                        dist = np.sqrt((ankX-ankX_)**2+(ankY-ankY_)**2)*cnf.DISCRETIZATION
                        if dist < 0.15:
                            isOverlap = True
                            break
                    if isOverlap:
                        continue
                dist2center = np.sqrt(((markerPixX-yBbox)-ankX)**2+((markerPixY-xBbox)-ankY)**2)
                ankleList.append([float(x+w/2), float(y+h/2), dist2center])
            ankleArray = np.array(ankleList)
            
            cv2.rectangle(imgBEV, (xBbox, yBbox), (xBbox+BboxWidth, yBbox+BboxHeight), 100, 1)
            if len(ankleList) > 0:
                for (ankX, ankY, _) in ankleArray[ankleArray[:,2].argsort()][:min(len(ankleList), 2)]:
                    centerX, centerY = int(xBbox+ankX), int(yBbox+ankY)
                    surrPix = 5
                    cv2.rectangle(imgBEV, (centerX-surrPix, centerY-surrPix), (centerX+surrPix, centerY+surrPix), 200, 1)
                initBbox = (centerX-surrPix, centerY-surrPix, 2*surrPix, 2*surrPix)
            else:
                initBbox = initBbox_
                
            if sum(1 for i in initBbox if i<0 or i>min(cnf.BEV_WIDTH, cnf.BEV_HEIGHT)) >0:
                isBboxDeclared = False
            else:
                isBboxDeclared = True
            
            posInfo['Found'] = 1
            posInfo['Dist'] = np.sqrt(markerPosX**2+markerPosY**2)
            posInfo['Angle'] = np.rad2deg(np.arctan2(markerPosY, markerPosX))
            
            #-------------------------------------------------#
            
        elif isBboxDeclared:
            if isTracMarkerEx.cur == False and isTracMarkerEx.pre == True:
                tcheck = Timer('I Initialize BEV based tracking')
                tracker = trackerInitializer()
                tracker.init(imgBEV, initBbox)               
                tcheck.Done(timeDict)
            tcheck = Timer('J BEV based tracking info')
            (success, boundingBox) = tracker.update(imgBEV)
            if success:
                xBbox, yBbox, BboxWidth, BboxHeight = (int(i) for i in boundingBox)
                
                markerX, markerY = pix2coor(float(yBbox+BboxHeight/2), float(xBbox+BboxWidth/2), (cnf.BEV_WIDTH, cnf.BEV_HEIGHT), cnf.DISCRETIZATION)
                posInfo['Found'] = 1 if success else 0
                posInfo['Dist'] = 100*np.sqrt(markerX**2+markerY**2) if success else 0
                posInfo['Angle'] = np.rad2deg(np.arctan2(-markerY, markerX)) if success else 0
                
                #For Human----------------------------------------#
                #cv2.imshow('Marker on LiDAR', cv2.resize(imgBEV[yBbox:yBbox+BboxHeight, xBbox:xBbox+BboxWidth],(400, 400)))
                cv2.rectangle(imgBEV, (xBbox, yBbox), (xBbox+BboxWidth, yBbox+BboxHeight), 255, 1)

                #-------------------------------------------------#
            
            
            
            tcheck.Done(timeDict)

        #For Human----------------------------------------#
        if args.showResult or args.saveVideo:
            tcheck = Timer('K BEV human interface')
            cv2.circle(imgBEV, tuple(map(int, cnf.CENTER)), int(cnf.RADIUS), 150, 1)
            cv2.line(imgBEV, tuple(map(int, cnf.CENTER)), tuple(map(int, cnf.CENTER+np.array([0, -2*cnf.RADIUS]))), 200, 1)
            cv2.line(imgBEV, tuple(map(int, cnf.CENTER)), tuple(map(int, cnf.CENTER+np.array([-cnf.RADIUS, 0]))), 200, 1)
            cv2.rectangle(imgBEV, (0, 0), (cnf.BEV_WIDTH-1, cnf.BEV_HEIGHT-1), 200, 2)

            tcheck.Done(timeDict)
        #-------------------------------------------------#
    
    
    GPIO.output(led_onCamera, GPIO.HIGH if isTracMarkerEx.cur else GPIO.LOW)
    if ('success' in globals() and not isTracMarkerEx.cur):
        GPIO.output(led_onLidar, GPIO.HIGH if success else GPIO.LOW)
    else:
        GPIO.output(led_onLidar, GPIO.LOW)
    
    msgPos = f'{posInfo["Found"]},{posInfo["Dist"]:.2f},{posInfo["Angle"]:.2f}'
    ser.write(msgPos.encode())    
    
    if args.showResult or args.saveVideo:
        tcheck = Timer('L Show all info')
        timer.cur = time.time()
        cv2.putText(imgCamera, f'FPS : {1/(timer.cur-timer.pre):.1f}', (10, 40), font, fontSize, (0, 255, 140), 2, cv2.LINE_AA)
        timer.pre = timer.cur
        
        imgBEVresize = cv2.cvtColor(cv2.resize(imgBEV, dsize=(int(cameraHeight), int(cameraHeight)), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        
        msgTrackingInfo =  f'Tracker : {trackerName}'
        cv2.putText(imgBEVresize, msgTrackingInfo, (10, 40), font, fontSize, (255, 255, 255), 2, cv2.LINE_AA)
        
        msgFound_lidar1 = 'Lidar based tracking : '+('ON' if (not isTracMarkerEx.cur and 'success' in globals()) else 'OFF')
        cv2.putText(imgBEVresize, msgFound_lidar1, (10, int(cameraHeight-30)), font, fontSize, (255, 255, 255), 2, cv2.LINE_AA)
        if ('success' in globals() and not isTracMarkerEx.cur):
            msgFound_lidar2 = 'Tracking '+('success' if success else 'fail')
            cv2.putText(imgBEVresize, msgFound_lidar2, (10, int(cameraHeight-60)), font, fontSize, (255, 255, 255), 2, cv2.LINE_AA)
        
        if posInfo['Found']:
            cv2.putText(imgBEVresize, f'PosInfo : R={posInfo["Dist"]:.2f}, deg={posInfo["Angle"]:.2f}', (10, int(cameraHeight-90)), font, fontSize, (255, 255, 255), 2, cv2.LINE_AA)
        
        msgFound_camera = 'Camera based tracking : '+('ON' if isTracMarkerEx.cur else 'OFF')
        cv2.putText(imgCamera, msgFound_camera, (10, 70), font, fontSize, (0, 255, 140), 2, cv2.LINE_AA)
        
        #imgCameraResize = cv2.resize(imgCamera, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        imgShow = np.hstack([imgCamera, imgBEVresize])
        
        #cv2.imshow('Image by Camera', imgCameraShow)
        #cv2.imshow('Image by LiDAR', imgBEVshow)
        cv2.imshow('Camera and LiDAR', imgShow)
        if args.saveVideo:
            frameArray.append(imgShow)
    
    k = cv2.waitKey(1)
    if k&0xFF == 27:
        break
    tcheck.Done(timeDict)
    isTracMarkerEx.pre = isTracMarkerEx.cur
    
    if args.saveTimeInfo:
        timeArray.append(timeDict)

print('Terminate main program')
cap.release()
cv2.destroyAllWindows()
laser.turnOff()
laser.disconnecting()
GPIO.output(led_onLidar, GPIO.LOW)
GPIO.output(led_onCamera, GPIO.LOW)
print('Done')

if args.saveTimeInfo:
    print('Save time data...', end='')
    with open('timeCheckData', 'wb') as f:
        pickle.dump(timeArray, f)
        
    print('Done')

if args.saveVideo:
    print(f'Save video to {os.path.join(args.videoPath, args.videoName)}')
    print(f'{len(frameArray)} Scenes')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    videoHeight, videoWidth = frameArray[0].shape[:2]
    videoOut = cv2.VideoWriter(os.path.join(args.videoPath, args.videoName), fourcc, 12, (videoWidth, videoHeight))

    for i, frame in enumerate(frameArray):
        videoOut.write(frame)
        if i%10 == 0:
            print('.', end='')
    videoOut.release()
    print('Done')


