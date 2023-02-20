from telnetlib import Telnet
import time
import struct
from threading import *
from keyboard import *

last_button = None

# creating a class with methods that contain basic drone functions and also to recieve sensory data
class drone(Thread):  
    throttle = 1000
    yaw_val = 1500
    pitch_val = 1500
    roll_val = 1500
    armed = None
    bool_pid = False
 
    def __init__(self, last_button=None):
        super(drone, self).__init__()
        pass

    # method to create an "IN" and "OUT" communication packets
    def make_in(self, command: int, byte_arr: bytes):  

        cmd = struct.pack(f"<cBB{len(byte_arr)}s", b'<',
                          len(byte_arr), command, byte_arr)

        crc = 0
        for c in cmd[1:]:
            crc ^= c
        crcb = bytes([crc])
        return b"$M" + cmd + crcb

    # Defining packet function with all stable values
    def msp_set_raw_rc(self, roll=1500, pitch=1500, throttle=1000, yaw=1500, aux1=2100, aux2=900, aux3=1500, aux4=1500):# This is a generalized method with equilibrium parameters which are altered to create      payload of individual packets
        payload = struct.pack("<8H", roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)
        return drone.make_in(self, 0xc8, payload)

    # Defining functions that change packet function parameters according to function
    def arm(self): 
        return drone.msp_set_raw_rc(self, throttle=1000, aux4=1500)

    def disarm(self): 
        return drone.msp_set_raw_rc(self, aux4=900)

    def takeoff(self):
        return drone.make_in(self, 0xd9, struct.pack("<H", 1))

    def land(self): 
        return drone.make_in(self, 0xd9, struct.pack("<H", 2))

    def roll(self): 
        return drone.msp_set_raw_rc(self, throttle=1500, roll=1700)

    def croll(self): 
        return drone.msp_set_raw_rc(self, roll=1300,)

    def pitch(self): 
        return drone.msp_set_raw_rc(self, throttle=1500, pitch=1600)

    def cpitch(self): 
        return drone.msp_set_raw_rc(self, throttle=1500, pitch=1400)

    def yaw(self):
        return drone.msp_set_raw_rc(self, throttle=1500, yaw=2000)

    def cyaw(self):
        return drone.msp_set_raw_rc(self, throttle=1500, yaw=1000)


    # Functions that send empty payload for recieving sensor values
    def msp_attitude(self):
        payload = bytearray(6)                                      # creating an empty 6 byte array (empty payload)
        return drone.make_in(0x6c, payload)

    def raw_imu(self): 
        payload = bytearray(18)                                     #creating an empty 18 byte array(empty payload) 
        return drone.make_in(0x66, payload)

    def msp_altitude(self): 
        payload = bytearray(6)                                      # creating an empty 6 byte array (empty payload)
        return drone.make_in(0x6d, payload)


    # the run method is overridden to define the thread body   
    def run(self):
        # establishing a connection using telnet protocol
        with Telnet('192.168.4.1', 23) as tn: 

            print("ready to take commands")
            try:
                while True:
                  # each keyboard input is assosiated with corresponding drone action, and input is taken from "keyboard.py"
                    last_button = keyboard.run(self)

                    if last_button == "p":
                        tn.write(self.disarm())
                        drone.armed = False

                    elif last_button == "o":
                        tn.write(self.arm())
                        drone.armed = True

                    elif last_button == "t" and drone.armed:
                        print('TAKEOFF START', end='\n', flush=True)

                        #Sending takeoff command and setting throttle to 1700
                        tn.write(self.takeoff()) 
                        drone.armed = True
                        drone.throttle = 1700
                        drone.bool_pid = True

                    elif last_button == "l" and drone.armed:
                        print('LAND START', end='\n', flush=True)
                        
                        #Sending land command and setting throttle to 1400
                        tn.write(self.land())
                        time.sleep(2)
                        drone.throttle = 1400
                        drone.armed = True
                        drone.bool_pid = False

                    elif last_button == "d" and drone.armed:
                        print('ROLL START', end='\n', flush=True)
                        
                        tn.write(self.roll())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.roll_val=1700
                        drone.armed = False
                        drone.bool_pid = False

                    elif last_button == "a" and drone.armed:
                        print('Counter roll started', end='\n', flush=True)
                        tn.write(self.croll())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.roll_val=1300
                        drone.armed = True

                    elif last_button == "w" and drone.armed:
                        print('PITCH STARTED', end='\n', flush=True)
                        tn.write(self.pitch())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.pitch_val=1600
                        drone.armed = True

                    elif last_button == "s" and drone.armed:
                        print('counter pitch started', end='\n', flush=True)
                        tn.write(self.cpitch())
                        drone.throttle = 1500
                        drone.pitch_val=1400
                        drone.armed = True

                    elif last_button == "." and drone.armed:
                        print('YAW STARTED', end='\n', flush=True)
                        tn.write(self.yaw())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.yaw_val=2000
                        drone.armed = True
                        
                    elif last_button == "," and drone.armed:
                        print('counter yaw started', end='\n', flush=True)
                        tn.write(self.cyaw())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.yaw_val=1000
                        drone.armed = True

                    elif last_button == "b" and drone.armed:
                        drone.throttle = 1500
                        drone.roll_val = 1500
                        drone.pitch_val = 1500
                        drone.yaw_val = 1500

                    if not drone.armed: 
                        # Disarms the drone when variable is false
                        tn.write(self.disarm())
                        print('DISARMED', end='\n', flush=True)
                        continue
                    
                    # Sending Throttle, Roll, Pitch, Yaw values to the drone using packet calling function
                    tn.write(self.msp_set_raw_rc(roll= int(drone.roll_val), pitch= int(drone.pitch_val), throttle=int(drone.throttle), yaw= int(drone.yaw_val)))
                    time.sleep(0.2)

                    # The following section is used to get sensor inputs from the drone

                    # Sending a packet with empty payload to recieve "OUT" packets from drone
                    tn.write(self.msp_attitude()) 

                    # reading the data packets sent by the drone
                    att_data_pack = tn.read_eager() 
                    att_data = att_data_pack.split(b'$M')

                    #unpacking the received attitude packet
                    try:
                        att_res = struct.unpack('<BBBBBB', att_data[1])
                    except:
                        pass
                    
                    #creating a text file to store attitude information
                    try:
                        with open("attitude_data.txt", "a+") as f:
                            f.write(str(att_res)+"\n")
                    except:
                        pass

                    tn.write(self.msp_altitude()) 
                    alt_data_pack = tn.read_eager() 
                    alt_data = alt_data_pack.split(b'$M')
                    try:
                        alt_res = struct.unpack('<BBBBBB', alt_data[1]) 
                    except:
                        pass

                    try:
                        with open("altitude_data.txt", "a+") as f:
                           f.write(str(alt_res)+"\n")
                    except:
                        pass

                    tn.write(self.raw_imu()) 
                    raw_imu_data_pack = tn.read_eager() 
                    imu_data = raw_imu_data_pack.split(b'$M')
                    try:
                        imu_res = struct.unpack('<BBBBBBBBB', imu_data[1])
                    except:
                        pass

                    try:
                        with open("raw_imu_data.txt", "a+") as f:
                            f.write(str(imu_res)+"\n")
                    except:
                        pass
                    print('throttle: {:4.2f}    roll: {:4}    pitch {:4}    yaw {:4}'.format(drone.throttle_val, drone.roll_val, drone.pitch_val, drone.yaw_val), end='\n', flush=True)
                    last_button = None

            except KeyboardInterrupt:
                pass
                tn.write(self.disarm())


d1 = drone() 
obj2 = keyboard() 
obj2.start() 
d1.start() 