# Project Overview
Computer vision system developed for detecting a landing/ drop-off location. Should someone decide to reproduce this project, a Jetson Nano, gimbal and Pixhawk flight controller are required (and of course the additional drone paraphernalia). Additionally, some experimentation with text detection was developed as an alternative method for detecting landing locations.

Additionally, I created a method of geotagging targets using only a Pixhawk, Jetson Nano and gimbal. This method is currently under construction and I will endevour to post more information on this at a later date. Some outputs are presented below however.

# Prototyping: 
During the early stages of development, I experimented with QR codes as a means of locating a target. Two methods I explored involved using an open source QR code reader which offered high frame rates and feature matching algorithms such as SURF, SIFT and ORB. Each had its own advantage, some pictures are presented below.
<p align="center">
   <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/QRSURF.png">
  <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/Proto1.png">
  <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/Proto2.png">
    <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/Example1.png">
</p>

# Text Detection:
I also experimented with using text detection algorithms and letter based targets.
<p align="center">
  <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/TextDetection.png">
</p>

# Dev Kit & Program Flow Diagram
Below is a top level presentation of the program architecture and the equipment I used during development. I also integrated a log file protocol which stored all the video feed of the flight. This meant that each flight would be recorded with its own unique timestamp. The start time of the flight was upon pixhawk arming and the end time was during disarm or during emergency (in order to compile the file before a potential power cut/ crash).
<p align="center">
   <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/DevKit.png">
  <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/ProgrammingFlowDiagram.png">
   <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/Logs.png">
</p>

# Geotagging
As you can see from the picture, when a target is found, the LAT,LON is allocated to it (this value is relative to the drone position and altitude). This method of geotagging to unique landing zones can act as reference points for the drone if GPS signal is cut out for any reason.
<p align="center">
  <img width="750" src="https://github.com/OliverHeilmann/Vision-System-Jetson-Nano-Pixhawk-Drone/blob/master/CV%20Github%20Pics/GPSAllocate.png">
</p>
