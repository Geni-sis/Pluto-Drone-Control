from Aruco import *
from plutodrone import *
import time

# Defining the height parameters
height_max= 3
height_min = 0 
height_set= 1 # Hovering hEight

# Update frequency ensures that the derivative and integral term in PID does not remain stuck at 0
udpate_frequency = 20

# To set the interval for Throttle, Roll, Pitch and Yaw values respectively
minm=[1400,1400,1400,1400]
maxm=[1800,1660,1600,1600]

throttle_hover = 1500

previous_error = np.zeros(4)
previous_error_sum = np.zeros(4)

# Setting the Kp, Kd and Ki values for Throttle, Roll, Pitch and Yaw respectively (Manual Tuning)
# Might need to change according to environment
kp=[30,6,6,10]
kd=[0,25,25,30]
ki=[0,0,0,0]

previous_error_height=0
dt=0.1

# Initializing
roll_temp = 0
pitch_temp = 0
throttle_temp = 0
previous_time = 0
fxd_tm=time.time()
error_sum = np.zeros(4)

while 1: 
    while drone.bool_pid:
        
        # Calculating the Error for movement in X,Y and Z directions along with Yaw orientations
        current_height= height_max-height.distance
        error_height =  height_set - current_height           
        error_roll= - height.y
        error_pitch= - height.x
        error_yaw= height.yaw
        error=[error_height , error_roll , error_pitch , error_yaw]

        #Limits the Integrl Term to counter LARGE UNWINDING
        if roll_temp <= 1420 or roll_temp >= 1530:
            error_sum[1] = 0
        if pitch_temp <= 1470 or pitch_temp >= 1530:
            error_sum[2] = 0
        if throttle_temp <= 1400 or throttle_temp >= 1800:
            error_sum[0] = 0

        error_sum = np.add(previous_error_sum, error)
            
        #Calculates Throttle Value using PID control algorithm    
        throttle_temp= 1500+ 5*(kp[0]*error[0]  +  kd[0]*((error[0]-previous_error[0])/dt)  +  ki[0]* (error_sum[0])*(dt))

        # PID for Roll, Pitch and Yaw kicks in only after a time of 3 seconds, which ensures smooth takeoff (increase if takeoff is unstable) 
        if (time.time()- fxd_tm)<=3:
            roll_temp=1500
            pitch_temp=1500
            yaw_temp=1500
        else:
            roll_temp = 1500+ 2*(kp[1]*error[1]  +  kd[1]*((error[1]-previous_error[1])/dt)  +  ki[1]* (error_sum[1])*(dt))
            pitch_temp = 1500- 2*(kp[2]*error[2]  +  kd[2]*((error[2]-previous_error[2])/dt)  +  ki[2]* (error_sum[2]*(dt)))
            yaw_temp = 1500 - 7*(kp[3]*error[3]  +  kd[3]*((error[3]-previous_error[3])/dt)  +  ki[3]* (error_sum[3])*(dt))

        previous_error=error
        previous_error_sum = error_sum

        # here Max and Min functions are used to ensure that all values remain between an interval as defined by the minm and maxm values
        throttle=max(minm[0], min(maxm[0],int(throttle_temp) ))
        roll=max(minm[1], min(maxm[1], roll_temp))
        pitch=max(minm[2], min(maxm[2], pitch_temp))
        yaw=max(minm[3], min(maxm[3], yaw_temp))
        
        # Sending calculated values to the drone
        drone.throttle=throttle
        drone.roll_val=roll
        drone.pitch_val=pitch
        drone.yaw_val=yaw

        if 0xFF == ord('q'):
            break
 
        time.sleep(1/udpate_frequency)