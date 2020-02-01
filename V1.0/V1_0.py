import numpy as np
import cv2, imutils
import os, csv, pdb,time
import math as m
import matplotlib.pyplot as plt
from ShapeDetect import ShapeDetector
from time import strftime, gmtime
from imutils.video import FPS
from Pixhawk_Jetson import PIX_GPS_IMU
from QR_SCAN_Thread import ScanQR

#Connect to CSI-2 Camera Port (if Video==None)
def gstreamer_pipeline (capture_width=3820, capture_height=2464, display_width=720, display_height=405, framerate=60, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))


#Connect to Either Webvideostream or Filevideostream. If Webcam, Record Footage
def VideoStream(sudoPassword,video,resolution):
    #Determine Type of Video Stream
    if video==None: #Webvideostream
        #Restart nvargus-daemon to Allow for Video Stream Connection
        command='sudo systemctl restart nvargus-daemon'
        os.system('echo %s|sudo -S %s' % (sudoPassword, command))

        #Then use Webcam
        from imutils.video import WebcamVideoStream
        try:
            vs = WebcamVideoStream(gstreamer_pipeline(capture_width=3820, 
                                              capture_height=2464, 
                                              display_width=resolution[0],
                                              display_height=resolution[1],
                                              framerate=60, 
                                              flip_method=0), 
                                              cv2.CAP_GSTREAMER).start()
            video_connection=True
        except:
            print('/n/nUNABLE TO OPEN CAMERA/n/n')
            vs=None; video_connection=False
    else:
        #Then use Video
        from imutils.video import FileVideoStream
        try: #Filevideostream
            vs = FileVideoStream(video).start()
            video_connection=True
        except:
            vs=False; video_connection=False
    return vs, video_connection


#Innitate Connection With Pixhawk
def Pixhawk(tty="/dev/ttyTHS1", baud=1500000):
    #Connect to Pixhawk via Serial (check port is the same as default).
    #Note: If system hangs, arm Pixhawk
    pix=PIX_GPS_IMU(tty, baud)
    connection=pix.connect()    
    #If No Connection Made, Try Again
    while connection!=True:
        print('Trying to Connect Again...')
        connection=pix.connect()
    pix.start()
    return pix,connection


#Create a Folder to Append Flight Data To
def CreateFolder(directory=None):
    try:
        if directory==None or directory=='datetime':
            directory = strftime("%d-%m-%Y %H-%M",gmtime())
        elif directory!=None:
            directory=str(directory) + strftime("%d-%m-%Y %H-%M",gmtime())
        current_dir=os.path.realpath(__file__).split(__file__.split('/')[-1])[0]
        path=current_dir+'Flights/'+directory
        if not os.path.exists(path):
            os.makedirs(path)
        return path+'/'
    except OSError:
        print('Error Creating Directory: {}'.format(directory))
        return './'
    

#Heads Up Display of Useful Parameters
def HUD(frame,gamma,resolution,LAT=0,LON=0,ALT=0,ROLL=0,PITCH=0,YAW=0,STATUS=None):    
    # Update the FPS counter
    fps.update(); fps.stop()
    
    #Radians to Degrees
    ROLL=round(m.degrees(ROLL),3)
    PITCH=round(m.degrees(PITCH),3)
    YAW=round(m.degrees(YAW),3)
    
    #Assign HUD Paramters
    labels=["FPS: {}","g: {}","Res: {}","ROLL: {}","PITCH: {}",
            "YAW: {}","LAT: {}","LON: {}","ALT: {}","DISCONNECTED{}"]
    values=[str(round(fps.fps(),2)),round(gamma,2),resolution,ROLL,
            PITCH,YAW,LAT,LON,round(ALT,1),' ']
    shape=frame.shape; text_size=0.25*float(shape[1]/shape[0])
    #print('\nFPS: '+str(round(fps.fps(),2))+'\nResolution: '+str(resolution))
    
    # Display HUD on Frame
    lox=10;loy=30;color=(0,0,255)
    if STATUS==True:
        labels=labels[0:-1]; values=values[0:-1]
        color=(255,0,255)
    for i in range(0,len(labels)):
        cv2.putText(frame, labels[i].format(values[i]),(lox,loy),
                    cv2.FONT_HERSHEY_SIMPLEX,text_size,color, 2);
        loy+=int((37.43*0.3*shape[1])/shape[0])


#Adjust Exposure of Frame
def Gamma(frame, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
	for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(frame, table)


#Apply Mask
def Mask(frame,iterations=1,Lcolor=np.array([140,60,50]),Hcolor=np.array([180,255,255])):
    #Convert Frame to HSV and Filter for Color Range
    hsv_vid=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)        
    mask=cv2.inRange(hsv_vid,Lcolor,Hcolor)
    
    #Dilate Mask
    kernel=np.ones((4,4),np.uint8)
    mask=cv2.dilate(mask,kernel,iterations)
    return mask


#Calculates Angle with Reference to Front of Drone (Called by TargetGPS)
def Angle(a, b, width):
    x1, y1 = a
    x2, y2 = b
    inner_product = x1*x2 + y1*y2
    len1 = float(m.hypot(x1, y1))
    len2 = float(m.hypot(x2, y2))
    theta=float(m.acos(inner_product/(len1*len2)))
    if cX_list[-1]<=width/2:
        theta=float((2*m.pi)-theta)
    return theta,len2


#Outputs a GPS coordinate for a Likely Match Target
def TargetGPS(frame,pix,cX,cY,PixhawkData):
    global pvec, GTrig
    #Collect CURRENT GPS,Attitude Coords
    LAT,LON,ALT,ROLL,PITCH,YAW=PixhawkData[0]
    
    #GPS Update Trigger
    if len(GTrig)<3:
        GTrig.extend([LAT,LON])#[ROLL,PITCH])
    else:
        GTrig=[]; GTrig.extend([ROLL,PITCH])
                    
    if len(GTrig)>3 and GTrig[0]-GTrig[2]!=float(0) and  GTrig[1]-GTrig[3]!=float(0):
        #Append Centriod of Target to Global List
        cX_list.append(cX);cY_list.append(cY)
        
        #Determine Frame Shape     
        height,width,_=frame.shape
        
        #Determine Text Size
        text_size=width/height
        
        #Apply tollerance to how far one expects target to move between frames
        #xtoll=m.ceil(0.2*frame.shape[0]); ytoll=m.ceil(0.2*frame.shape[1])
        try:
            #Draw line from centre of frame to centroid
            cv2.line(frame,(int(width/2),int(height/2)),
                     (int(cX_list[-1]),int(cY_list[-1])),(0,255,0),2)
            
            #Calc Angle
            a=(0,-height/2)
            b=((cX_list[-1]-width/2),(cY_list[-1]-height/2))      
            theta,lenb=Angle(a,b,width)   #lenb is the vector b length

            #Store Value of Angle
            if pvec==None:
                pvec=(lenb,theta,(LAT,LON,ALT,ROLL,PITCH,YAW))    
            else:
            #elif abs(cX_list[-1]-cX_list[-2])<xtoll and abs(cY_list[-1]-cY_list[-2])<ytoll:
                #Angle Target has Moved +- Rotation of Drone (-ve or +ve)
                dangle=abs(float(theta)-float(pvec[1]))+abs(float(YAW-pvec[2][5]))
                
                #Consider Condition Where Angles Are >=180 deg (Cosine Rule Breaks Down)
                if dangle>m.pi and float((2*m.pi)-dangle)!=m.pi:
                    dangle=float((2*m.pi)-dangle)
                    dPIX_app=m.sqrt(lenb**2+pvec[0]**2-(2*lenb*pvec[0]*m.cos(dangle))) #cosine rule
                elif dangle==m.pi or float((2*m.pi)-dangle)==m.pi:
                    dPIX_app=lenb+pvec[0]
                else:
                    dPIX_app=m.sqrt(lenb**2+pvec[0]**2-(2*lenb*pvec[0]*m.cos(dangle))) #cosine rule
                    
                #dPIX Altitude Corrected. Note that a relationship between GPS 
                #and Pixels will be made on one plane only. I.e. the ratio may
                #only be used on FrameN.
                if pvec[2][2]<ALT:
                    #This is climb scenario ie lenb shorter than should be
                    dPIX_rl=m.sqrt(dPIX_app**2 + (abs(ALT-pvec[2][2]))**2)
                    #print('CLIMB')
                elif pvec[2][2]>ALT:
                    #This is climb scenario ie lenb longer than should be
                    dPIX_rl=m.sqrt(dPIX_app**2 - (abs(ALT-pvec[2][2]))**2)
                    #print('DROP')
                else:
                    dPIX_rl=dPIX_app
                    #print('SAME')
                
                #Find dGPS
                dGPS=m.sqrt(abs(pvec[2][0]-LAT)**2 + abs(pvec[2][1]-LON)**2)
                
                #dGPS to dPIX ratio used for Target Measurement
                ratio=float(dGPS/dPIX_rl)
                
                #True North @ Heading = 0. Use this to find dLAT,dLON and add 
                #these values to the original GPS coords from the centre of
                #frameN.
                dLAT1=float((ratio*pvec[0]) * m.cos(pvec[2][3]+pvec[1])) #dLAT (FrameN)
                dLON1=float((ratio*pvec[0]) * m.sin(pvec[2][3]+pvec[1])) #dLON (FrameN)
                TLAT=pvec[2][0]+dLAT1
                TLON=pvec[2][1]+dLON1
                
                #Final Location Estimate/ Store for Filtering
                TARGET_LOC=[TLAT,TLON]
                if TARGET_LOC[0]!=0 and TARGET_LOC[1]!=0:
                    GPS_Y_Store.append(TLAT)
                    GPS_X_Store.append(TLON)
                
                
                #Visual Representation of Where Target Has Moved to on Frame
                cv2.circle(frame, (cX_list[-2],cY_list[-2]), 3, (255,0, 255), -1)
                cv2.line(frame,(int(cX_list[-2]),int(cY_list[-2])),
                         (int(cX_list[-1]),int(cY_list[-1])),(255,0,255),3)
                
                cv2.putText(frame,"LAT: {}, LON: {}".format(TLAT,TLON),
                            (int(cX_list[-2]),int(cY_list[-2])),
                            cv2.FONT_HERSHEY_SIMPLEX,0.65*text_size,
                            (255,0, 255), 3)
                #print('\n\n dAngle:{} \n dPIX_app:{} \n PIX_rl:{} \n dGPS:{} \n Ratio:{}'
                #      .format(m.degrees(dangle),dPIX_app,dPIX_rl,dGPS,ratio))
                #print('TARGET ESTIMATE: {}'.format(TARGET_LOC))
                #print('DRONE LOCATION: {}\n\n'.format([pvec[2][0],pvec[2][1]]))
                #Assign Current Frame Info to be New Previous Vector Info
                pvec=(lenb,theta,(LAT,LON,ALT,ROLL,PITCH,YAW))
                return [LAT, LON, ALT, ROLL, PITCH, YAW, TLAT, TLON]
        except:
            #Return Zero Values to Alert Script of Error
            return [0,0,0,0,0,0,0,0]
    

#Marks Likely Target by Looking for Square within Squre
def TargetScanner(frame,new_frame,mask,processing_width,pix,scan,PixhawkData,STATUS):
    #Determine Frame Shape     
    height,width,_=new_frame.shape
    
    #Determine Text Size
    text_size=width/height
    
    #Used to save passed contours for pointPolygonTest
    CoM=[];contours=[];areas=[];
    
    #Triggers for Modules
    trigger=0
    
    #Find Contours
    cnts = cv2.findContours(mask, cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    # Loop over the contours for size of shapes that have passed
    for c in cnts:
        #If Contour Area>threshold then Proceed
        area=cv2.contourArea(c)
        if area>int(0.13*processing_width):
            #If Contour is Rectangle/Square, Proceed
            sd = ShapeDetector(); shape = sd.detect((c))
            if shape=='square' or shape=='rectangle':                        
                #Find Rectangle centroid
                M = cv2.moments(c)
                cX = int(M["m10"]/ M["m00"])
                cY = int(M["m01"]/ M["m00"])

                #Check whether one centroid lies within another
                #Green box indicates two boxes detected i.e. higher confidence
                CoM.append((cX,cY));contours.append(c); areas.append(area)
                for i in CoM:
                    #If Centroid of Rectangle Inside of Another then Proceed
                    if cv2.pointPolygonTest(c,i,False)==1 and len(CoM)>1:
                        max_value = max(areas)
                        max_index = areas.index(max_value)
                        
                        #Create new bounding box dimensions
                        x, y, w, h = cv2.boundingRect(contours[max_index])
                        #bbox = (x,y,w,h)

                        #Drawing contours of passed sections
                        cv2.drawContours(new_frame, [contours[max_index]], -1, (20,255,0), 3)
                        
                        #Get/Show ROI
                        roi = frame[y:y + h, x:x + w]
                        roi=imutils.resize(roi,250)
                        cv2.imshow('ROI Image',roi)
                        
                        #Feed ROI to QR Scanner Thread, Check For True Result
                        scan.feed(roi)
                        result=scan.fetch()
                        if result[1]!=False and result[0]==True:
                            cv2.putText(new_frame,"Target Found: {}".format(result[1]),
                                        (x,y),cv2.FONT_HERSHEY_SIMPLEX,0.65*text_size,
                                        (0, 255, 0), 3)
                        elif result[1]!=False and result[0]==False:
                            cv2.putText(new_frame,"Other Target: {}".format(result[1]),
                                        (x,y),cv2.FONT_HERSHEY_SIMPLEX,0.65*text_size,
                                        (0, 255, 0), 3)
                        else:
                            cv2.putText(new_frame,"Likely Match", (x,y),
                                        cv2.FONT_HERSHEY_SIMPLEX,0.65*text_size,
                                        (0, 255, 0), 3)

                        #Trigger TargetGPS & Track_Obj after Likely Match
                        trigger+=1
                                                
                        #Triggers Multiple Object Tracker
                        #trig_MOT=True
                        #Trigger GPS Detector
                        #trig_GPS=True
                    else:
                        cv2.drawContours(new_frame, [c], -1, (0,0,0), 3)
                
                #Determine GPS Coordinates of Likely Match
                if trigger>=2 and STATUS==True:
                    #Pass Values to TargetGPS Function 
                    state=TargetGPS(new_frame,pix,cX,cY,PixhawkData)
                    trigger=0
    
    if not 'state' in locals() or state == None:
        if STATUS==True:
            #Collect CURRENT GPS,Attitude Coords
            state=PixhawkData[0]; state.extend((0.0,0.0))
        elif STATUS!=True:
            #If Connection is False, Return None
            state=[0,0,0,0,0,0,0,0]
    return state
 

#Plot the LAT and LON Results from Flight
def Save_Plot(path,LON,LAT,itt=1,plot=None):
    #Save Results
    with open(path+'GPS_Coords_{}.csv'.format(str(itt)), 'w') as f:
            writer = csv.writer(f)
            writer.writerows(zip(LON,LAT))   
    #Plot Results
    if plot!=None or plot==True:
        if len(LON)==0:
            LON=[0.0];LAT=[0.0]
        plt.plot(LON,LAT)
        plt.ylabel('LONGITUDE'); plt.xlabel('LATITUDE')
        plt.show()

    
#Main Script
def Main(sudoPassword='radex',video=None, gamma=1, tracker_no=5,resolution=(1280,720), new_frame_width=None, tty="/dev/ttyTHS1", baud=1500000,search='RADEX QR Code A1'):
    global fps, cX_list, cY_list, GPS_X_Store, GPS_Y_Store,pvec, GTrig
        
    #Connect to Either Webvideostream or Filevideostream
    vs,video_connection=VideoStream(sudoPassword,video,resolution)
    
    #Establish Connection with Pixhawk (Thread)
    pix, connection=Pixhawk(tty, baud)
    
    #Initiate QR Scanner (Thread)
    scan=ScanQR(search)
    scan.start()
    fps = FPS().start()
    try:
        while True:
            #Collect Pixhawk Armstate
            armcheck=pix.fetch()[1][2]
            #If Drone Is Armed...
            if armcheck>=float(128):
                #Define Lists for Value Storage
                cX_list=[]; cY_list=[]
                GPS_X_Store=[]; GPS_Y_Store=[]
                pvec=None; GTrig=[]
                
                #Create Folder for Appending Flight Data...
                path=CreateFolder(directory='datetime')
                
                #Ensure Connection to Pixhawk Made Before Progressing
                if connection==True and video_connection==True:
                    #Start FPS Monitor
                    Vtrigger=True; Ptrigger=True; itt=1
                    while armcheck>=float(128):
                        #Read frame- threaded
                        frame=vs.read()
                        if video!=None:
                            frame=imutils.resize(frame, width=resolution[0])  
                        #Collect CURRENT GPS,Attitude Coords, Armstate and Connectivity
                        PixhawkData=pix.fetch()
                        armcheck=PixhawkData[1][2]
                        STATUS=pix.checkstatus()
                        
                        height,width,_=frame.shape
                            
                        #Create Copy of Original Frame for Display
                        new_frame=frame.copy()
                        
                        #Apply Contrast Correction to Processing Frame
                        frame=Gamma(frame,gamma=gamma)
                
                        #Create Mask of Processing Frame
                        mask=Mask(frame,1)    #filter for green
            
                        #Detect Likely Match in Frame
                        S=TargetScanner(frame,new_frame,mask,resolution[0],pix,scan,PixhawkData,STATUS)
                        
                        #Draw Appropriate Grid & Indicate Centre  
                        cv2.line(new_frame,(0,int(height/2)),(int(width),int(height/2)),(0,0,0),2)
                        cv2.line(new_frame,(int(width/2),0),(int(width/2),int(height)),(0,0,0),2)
                        cv2.circle(new_frame, (int(width/2), int(height/2)), 5, (0, 255, 0),-1)
                        
                        #Display processed video feed in smaller frame size
                        if new_frame_width!=None:
                            #frame=imutils.resize(frame,new_frame_width)
                            new_frame=imutils.resize(new_frame,new_frame_width)
                            mask=imutils.resize(mask,new_frame_width)
                        
                        #Display Useful Params in HUD
                        HUD(new_frame,gamma,resolution,S[0],S[1],S[2],S[3],S[4],S[5],STATUS)
                        
                        #Setup VideoWriter. Frame Size Must be the SAME as Appended Frame. 
                        #Trigger Ensures this only Happens Once
                        if Vtrigger==True and STATUS==True:
                            height,width,_=new_frame.shape; Vtrigger=False
                            record = cv2.VideoWriter(path+'FlightVideo_1.avi', cv2.VideoWriter_fourcc(*'X264'),float(10),(width,height),True)
                            
                        #Save Frame For Data Collection
                        if Vtrigger==False or Ptrigger==False:
                            record.write(new_frame)
                        
                        #If Pixhawk is Disconnected, Automatically Save Flight Data
                        if STATUS==False and Vtrigger==False and Ptrigger==True:
                            Save_Plot(path,GPS_X_Store,GPS_Y_Store,itt=itt,plot=None) 
                            record.release()
                            #Carry on Recording but Do Not Overwrite Previous File
                            height,width,_=new_frame.shape
                            record = cv2.VideoWriter(path+'FlightVideo_2.avi', cv2.VideoWriter_fourcc(*'X264'),float(10),(width,height),True)
                            itt+=1; Ptrigger=False
                        
                        #cv2.imshow('Frame',frame)
                        cv2.imshow("New Frame",new_frame)
                        #cv2.imshow("Mask",mask)
                        
                        key=cv2.waitKey(1)
                        if key & 0xFF == ord('q'):
                            break      
                    #Save Data from Flight
                    Save_Plot(path,GPS_X_Store,GPS_Y_Store,itt=itt,plot=None) 
                    record.release()
            elif armcheck<float(128):
                print('Pixhawk Disarmed, Waiting for Arm Command to Begin')
                time.sleep(2)
    except KeyboardInterrupt:
        Save_Plot(path,GPS_X_Store,GPS_Y_Store,itt=itt,plot=None)
        record.release()
            
    #Cleanup at End
    print('CLOSING THREADS')
    fps.stop()
    pix.stop()
    scan.stop()
    vs.stop()
    cv2.destroyAllWindows()

#Run as Main Script (see main() for all input parameters)        
if __name__=='__main__':
    #Wait for System Boot Before Running Script
    time.sleep(10)
    
    #Trun Fan On for Program Duration
    os.system("sudo sh -c 'echo 65 > /sys/devices/pwm-fan/target_pwm'")
    #Main(video='/home/radex/Documents/OpenCV/RADEX/24_07_2019_Flight_MQR.mov',gamma=0.7,new_frame_width=900)
    Main(video=None,gamma=0.7,new_frame_width=700)
    
    #Reduce Fan Speed
    os.system("sudo sh -c 'echo 45 > /sys/devices/pwm-fan/target_pwm'")
