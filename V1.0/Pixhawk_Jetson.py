#   Created by Oliver Arent Heilmann
import threading,pdb,os
from pymavlink import mavutil
import sys,signal,time
'''
THIS SCRIPT INNITIATES A CONNECTION WITH A PIXHAWK VIA THE TTYTHS1 PORT AND
ALLOWS FOR THE COLLECTION OF GPS OR ATTITUDE PARAMTERS WHENEVER CALLED. THIS 
THREADED APPROACH ALLOWS FOR MINIMAL LATENCY BETWEEN JETSON AND PIXHAWK (AS 
WELL AS DATA PILE-UPS).

COMMANDS TO INITIATE PIXHAWK CONNECTION

Pixhawk = PIX_GPS_IMU(tty="/dev/ttyTHS1", baud=115200)
connect=Pixhawk.connect()
if connect==True:
    Pixhawk.start() # start thread
Pixhawk.stop()

NOTE: ENSURE THAT MAVPROXY IS RUNNING IN ORDER TO ESTABLISH LINK ON BOOT. FOUND
IN 
'''
def validate_msg(msg):
    if msg==None:
        return False
    elif not msg or msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()
            return False
    else:
        return True

def handler(signum, frame):
    raise Exception("\n\nERROR: Pixhawk Connection\n")

class PIX_GPS_IMU(threading.Thread):
    def __init__ (self,tty="/dev/ttyTHS1", baud=115200):
        #Hold Thread
        self.hold=threading.Event()
        #Other Thread Params Required
        self.tty=tty; self.baud=int(baud); self.master=0
        self.trigger=True; self.t0=float(0); self.status=False
        #Global GPS Params
        self.LAT_STR=0; self.LON_STR=0; self.ALT_STR=0
        #Global ATTITUDE Params
        self.ROLL_STR=0; self.PITCH_STR=0; self.YAW_STR=0
        #Global HEARTBEAT Params
        self.type=0; self.autopilot=0; self.base_mode=0
        self.custom_mode=0; self.system_status=0; self.mavlink_version=0
        #Thread State Checking
        self.has_been_called=False
        self.terminationRequired = False
        threading.Thread.__init__(self)
    
    def fetch(self):
        STATE=[float(self.LAT_STR),float(self.LON_STR),float(self.ALT_STR),
                 float(self.ROLL_STR),float(self.PITCH_STR),float(self.YAW_STR)]
        HEARTBEAT=[self.type,self.autopilot,self.base_mode,self.custom_mode,self.system_status,self.mavlink_version]
        return [STATE,HEARTBEAT]
    
    def checkstatus(self):
        return self.status
    
    def checkconnect(self,check1,check2,check3):
        if check1==False and check2==False and check3==False:
            if self.trigger==True:
                self.t0=time.time()
                self.trigger=False
            elapse=time.time()-self.t0
            if elapse>=float(7):
                print('Connection to Pixhawk Broken, Attempting to Reconnect...')
                self.trigger=True
                self.status=False
        else:
            self.t0=time.time()
            self.trigger=True
            self.status=True    #Used to Alert Main Script of Connection Status
    
    def stop(self):
            self.terminationRequired = True
            print ("\nPixhawk Connection Stopping")
            
    def connect(self):
        #Raise Timeout Condition for Pixhawk Connection
        signal.signal(signal.SIGALRM, handler)
        signal.alarm(10)
        #Initiate Serial Connection With Pixhawk
        self.master = mavutil.mavlink_connection(self.tty,self.baud)
        try:
            print("\n\nWaiting for APM heartbeat")
            self.master.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (self.master.target_system,self.master.target_system))
            signal.alarm(0) #cancel alarm
            #Establish MavProxy Connection
            os.system('sudo mavproxy.py --master={} --baudrate {} --aircraft MyCopter'.format(self.tty,self.baud))
            #Check that Pixhawk is Sending Non-Zero Data
            check=False
            while check==False:
                check=validate_msg(self.master.recv_match(type="ATTITUDE",blocking=False))
                print('    Begin Mavproxy Connection to Allow Data Transfer to Jetson')
                time.sleep(2)
            #Passed All Stages
            print('Connection Complete\n\n')
            return True
        except Exception as exc:
            print("\n\nPixhawk Connection Timed Out After 10 s")
            return False
            
    def run(self):
        #Start Data Collection
        while (not self.terminationRequired):
            try:
                # Get GPS coords
                GPS = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                check1=validate_msg(GPS)
                
                if check1==True:
                    self.LAT_STR = str(GPS.lat*(10**-7))
                    self.LON_STR = str(GPS.lon*(10**-7))
                    self.ALT_STR = str(GPS.alt*(10**-5))
                    #print('LAT: {}, LON: {}, ALT: {}'.format(self.LAT_STR,self.LON_STR,self.ALT_STR))
                
                # Get ATTITUDE coords
                ATTITUDE = self.master.recv_match(type="ATTITUDE", blocking=False)
                check2=validate_msg(ATTITUDE)
                if check2==True:
                    self.ROLL_STR = str(ATTITUDE.roll)
                    self.PITCH_STR = str(ATTITUDE.pitch)
                    self.YAW_STR = str(ATTITUDE.yaw)
                    #print('ROLL: {}, PITCH: {},YAW: {}\n'.format(self.ROLL_STR,self.PITCH_STR,self.YAW_STR))
                
                # Get HEARTBEAT for MODE Bitmap    
                HEARTBEAT=self.master.recv_match(type='HEARTBEAT',blocking=False)
                check3=validate_msg(HEARTBEAT)
                if check3==True:
                    self.Htype=HEARTBEAT.type
                    self.autopilot=HEARTBEAT.autopilot
                    self.base_mode=HEARTBEAT.base_mode
                    self.custom_mode=HEARTBEAT.custom_mode
                    self.system_status=HEARTBEAT.system_status
                    self.mavlink_version=HEARTBEAT.mavlink_version
                    HEARTBEAT=[self.Htype,self.autopilot,self.base_mode,self.custom_mode,self.system_status,self.mavlink_version]
                    
                #Pass Check1,2,3 States to Check for Connection
                self.checkconnect(check1,check2,check3)
            except Exception as e:
                error = str(e)
                self.master.write(error.encode())
        self.master.close()  #close Pixhawk connection when triggered by stop()