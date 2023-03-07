# Inter-IIT

The following is our solution to the Drona Aviation PS as a part of Inter IIT Tech Meet 11.0-

Task 1 has two files, namely, keyboard.py and plutodrone.py. The keyboard file uses MSVCRT to read keyboard inputs, while the pluto drone is the wrapper for drone control that sends signals based on these inputs.
  
The keyboard commands for the same are as follows- w- Pitch a- Counter Roll s- Counter Pitch d- Roll.

Task 2 requires three files, Aruco, Hover, and Rectangle. 
 
The first is for pose estimation of the drone with the help of a "RealSense" camera installed on the ceiling, while "Hover" uses a PID control algorithm for hovering the drone at a particular height (1 meter). You may change this value by changing the "height_set" variable. "Rectangle.py," similar to "Hover," uses PID control to move the drone rectangularly.
