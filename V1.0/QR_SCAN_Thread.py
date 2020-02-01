import cv2, threading
import pyzbar.pyzbar as pyzbar
'''
THIS SCRIPT IS DESIGNED TO SCAN INCOMING IMAGES SENT FROM THE MAIN SCRIPT.
QUEUES OF TASKS CAN BE GENERATED AND THIS THREAD WILL STORE & SCAN THEM 
SEQUENTIALLY. IF A SUCCESSFUL MATCH IS MADE, THE MAIN SCRIPT IS ALERTED.

COMMANDS TO INITIATE QR SCANNER CONNECTION

scan=ScanQR(search='RADEX QR Code for 255_0_255 BGR Color Detection')
scan.start()    #start thread
scan.feed(roi)  #feed new roi to thread
result=scan.fetch()    #fetch result from thread
scan.stop()     #close thread
'''
class ScanQR(threading.Thread):
    def __init__ (self,search=None):
        #Thread Params Required
        self.hold=threading.Event()
        self.result=[False,False]
        self.search=search; self.tasks=[]
        self.terminationRequired = False
        threading.Thread.__init__(self)
    
    #Decode Image, Return QR Datastring if Successful 
    def decode(self,img): 
        #Image Processing 
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find Barcodes and QR codes
        decodedObjects = pyzbar.decode(gray)
        if len(decodedObjects)>0:
            code=str(decodedObjects[0][0]).split("'")[1]
            return decodedObjects,code
        else:
            return False,False
    
    def feed(self,roi):
        self.tasks.append(roi)
        self.hold.set()
        
    def fetch(self):
        return self.result
    
    def stop(self):
            self.terminationRequired = True
            print ("QR Scanner Stopping\n")
            
    def run(self):
        #Start Scanning for Your QR
        while (not self.terminationRequired):
            try:
                #If No Tasks, Thread Hangs
                if len(self.tasks)<=0:
                    self.hold.wait()
                #Continue With Tasks
                else:
                    _,code=self.decode(self.tasks[0])
                    #Pass T/F States to Alert Main Script of Successful Match
                    if code==self.search:
                        self.result=[True,code]
                    elif code!=False:
                        self.result=[False,code]
                    else:
                        self.result=[False,False]
                    #If Task, Prepare to Hang
                    if len(self.tasks)==1:
                        self.hold=threading.Event()
                    self.tasks.remove(self.tasks[0])      
            except Exception as e:
                error = str(e)
                print ('\n\nError in QR_Scan_Thread.py \n{}\n\n'.format(error))