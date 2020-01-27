#...........Drive a Servo motor with sin-wave PWM signal............
#Author: James Bramante
#Date: May 3, 2017

from Adafruit_PWM_Servo_Driver import PWM
import threading
import time
import math
import numpy as np

class RotorDriver(object):

    def __init__(self, channels = [0], magnitude = 0.5, period = 4., runtime = 60., comint = 1e-2, periodMultiple = 1):
        """ Constructor

        :type channels:  list
        :type magnitude: float
        :type period:    float
        :type runtime:   int
        :type comint:   float

        :param channels:  which channels to control rotors on
        :param magnitude: % of maximum rotor power
        :param period:    period of the sin wave, in seconds
        :param runtime:   how long to run the rotors for, in seconds
        :param comint:   desired refresh rate of commands to rotors
        """

        #Parse the input parameters
        self.channels = channels
        self.mag = np.array([magnitude,magnitude])           
        self.per = period
        self.runtime = runtime
        self.comint = comint
        self.multiple = int(periodMultiple)

        #FINAL variables
        self.PWMFREQ = 250 #Frequency of PWM signal in Hz
        self.NUMTICK = 4096 #Resolution of signal (i.e. 4096 = 12 bit)
        self.ZEROSPD = [1577,1573] #PWM-width, in microseconds, of 0 servo movement [Rotor 0 (arm without pressure sensor: dead range: [1551,1603]; Rotor 1: dead range: [1546,1599]]
        self.MAXSPD = [int(round(1990 * self.ZEROSPD[x]/1500)) for x in range(2)] #PWM-width, in microseconds of maximum forward speed
        self.MINSPD = 1100 #PWM-width, in microseconds of maximum reverse speed
        self.SERVO_ADDRESS = 0x40 #Address of I2C servo in hex
        self.BIAS = 0 #Bias with which to adjust value inside cos
        self.MULT2A = -0.2056
        self.MULT2B = -1.096
        self.MULT2C = 0.3652
        self.MULT3A = -0.2127
        self.MULT3B = -1.134
        self.MULT3C = 0.3781
        #These coefficients are used to alter the pattern of rotor output
        #to force the apparatus to oscillate at periods greater than
        #the system's harmonic period
        self.COEFFICIENTS = [[1,0,0],[self.MULT2A,self.MULT2B,self.MULT2C],[self.MULT3A,self.MULT3B,self.MULT3C]]

        #Additional variables
        self.usPerTick = 1./self.PWMFREQ * 1e6 / self.NUMTICK #microseconds per tick
        self.spdRange = [400,400]
        self.spdRange[0] = max(self.MAXSPD[0] - self.ZEROSPD[0], self.ZEROSPD[0] - self.MINSPD)
        self.spdRange[1] = max(self.MAXSPD[1] - self.ZEROSPD[1], self.ZEROSPD[1] - self.MINSPD)
        if self.per == 0:
            self.freq = 0
            self.BIAS = 0 
        else:
            self.freq = 2. * math.pi / self.per
            
        self.running = False
        self.flipper = 1.
        self.startTime = 0

        #Initialize the rotors
        self.pwm = PWM(self.SERVO_ADDRESS)
        self.pwm.setPWMFreq(self.PWMFREQ)
        self.pwm.setAllPWM(0,math.floor(self.ZEROSPD[0]/self.usPerTick))

    def daemonize(self):
        #Now daemonize the run function
        self.thread = threading.Thread(target=self.run,args=(),daemon=True)
        self.thread.start()


    #The threaded method that runs the rotors in the background
    def run(self):

        #Record the start time from which to determine when to stop
        self.startTime = time.perf_counter()
        self.running = True
        
        while self.running:
            #Determine new speed as a sin function
            tim = time.perf_counter()
            newSpeed = np.array(self.spdRange) * (self.COEFFICIENTS[self.multiple-1][0] * math.cos(self.freq * (tim-self.startTime) + self.BIAS) + self.COEFFICIENTS[self.multiple-1][1] * math.cos(self.freq * (tim-self.startTime) + self.BIAS)**3 + self.COEFFICIENTS[self.multiple-1][2] * math.cos(self.freq * (tim-self.startTime) + self.BIAS)**5)
            for channel in self.channels:
                #Direction of the rotor controlled by whether channel is even (-) or odd (+)
                self.pwm.setPWM(int(channel),0,math.floor((self.ZEROSPD[int(channel%2)]+math.copysign(1,self.flipper)*math.copysign(1,channel%2-1)*newSpeed[int(channel%2)]*float(self.mag[int(channel%2)]))/self.usPerTick))
                #self.pwm.setPWM(int(channel),0,math.floor((self.ZEROSPD[int(channel%2)]+math.copysign(1,self.flipper)*math.copysign(1,channel%2-1)*newSpeed[int(channel%2)])/self.usPerTick))

            #If we come to the intended end of the run, stop the rotors and exit loop 
            if ((time.perf_counter() - self.startTime) > self.runtime):
                self.pwm.setAllPWM(0,math.floor(self.ZEROSPD[0]/self.usPerTick))
                self.running = False
        
            time.sleep(self.comint)
        if (self.freq==0):
                self.flipper = -self.flipper

    def stop(self):
        self.running = False
        self.pwm.setAllPWM(0,math.floor(self.ZEROSPD[0]/self.usPerTick))

    def setPeriod(self,per):
        self.per = per
        if self.per == 0:
            self.freq = 0
            self.BIAS = 0 
        else:
            self.freq = 2. * math.pi / self.per
            self.BIAS = 0

    def setMultiple(self,mult):
        self.multiple = int(mult)

    def setMagnitude(self,mag):
        if (isinstance(mag,list) | isinstance(mag,tuple) | isinstance(mag,np.ndarray)):
            self.mag[0] = mag[0]
            self.mag[1] = mag[1]
        else:
            if np.max(self.mag) > 0:
                self.mag = np.array([self.mag[0]/np.max(self.mag)*mag,self.mag[1]/np.max(self.mag)*mag])
            else:
                self.mag = np.array([mag,mag])

    def setRuntime(self,tim):
        self.runtime = tim

    def setComint(self,com):
        self.comint = com

    def setChannels(self,chan):
        self.channels = chan
        self.pwm.setAllPWM(0,math.floor(0/self.usPerTick))

    def addChannels(self,chan):
        if isinstance(chan,list):
            self.channels += chan
        else:
            self.channels += [chan]
        self.pwm.setAllPWM(0,math.floor(0/self.usPerTick))
            
    def remChannels(self,chan):
        if not isinstance(chan,list):
            chan = [chan]
        self.channels = [x for x in self.channels if x not in chan]
        self.pwm.setAllPWM(0,math.floor(0/self.usPerTick))
        




        
        
        
    
