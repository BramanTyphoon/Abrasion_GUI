#...................Drive a pressure/temperature sensor..................
#Author: James Bramante
#Date: May 8, 2017

import ms5837
import threading
import time
import statistics as stats

class SensorDriver(object):

    def __init__(self, samplerate = 100., density = 997., baseTime = 0.):
        """ Constructor

        :type samplerate: float
        :type density: float

        :param samplerate: rate at which to sample the pressure sensor, in Hz
        :param density: density of the water, kg m^-3
        """

        #Parse the input parameters
        self.samplerate = samplerate
        self.density = density

        #FINAL variables
        self.I2CBUS = 1
        self.GRAV = 9.81

        #Initialize the sensor
        self.sensor = ms5837.MS5837_30BA()
        test1 = self.sensor.init()
        test2 = self.sensor.read()
        self.initialized = test1 & test2
        self.running = False

        #Initialize output variables
        if (baseTime == 0.):
            self.baseTime = time.time()
        else:
            self.baseTime = baseTime
            
        self.basepress = []
        for x in range(1,10):
            self.sensor.read()
            self.basepress += [self.sensor.pressure()]
            time.sleep(0.3)
        self.basepress = stats.mean(self.basepress)
        self.time = [time.time()-self.baseTime]
        self.temp = [self.sensor.temperature()]
        self.press = [self.basepress]
        self.depth = [0]

    def daemonize(self):
        #Daemonize the run function and thread it
        self.thread = threading.Thread(target=self.run,args=(),daemon=True)
        self.thread.start()

    #Collects sensor data in the background
    def run(self):

        self.running = True
        while self.running:
            #Produce pressure and temperature and depth data
            self.sensor.read()
            self.time += [time.time()-self.baseTime]
            self.temp += [self.sensor.temperature()]
            self.press += [self.sensor.pressure()]
            self.depth += [(self.sensor.pressure() - self.basepress)/self.density/self.GRAV]

            #Wait designated interval
            time.sleep(1/self.samplerate)

    #Stop sampling data
    def stop(self):
        self.running = False

    #Reset the sensor and start recording again, but keep parameters
    def reset(self):
        self.running = False

        #Re-initialize the sensor
        self.sensor = ms5837.MS5837_30BA()
        self.sensor.init()

        #Re-initialize everything
        self.baseTime = time.time()
        self.basepress = []
        for x in range(1,10):
            self.sensor.read()
            self.basepress += [self.sensor.pressure()]
            time.sleep(0.3)
        self.basepress = stats.mean(self.basepress)
        self.time = [time.time()-self.baseTime]
        self.temp = [self.sensor.temperature()]
        self.press = [self.basepress]
        self.depth = [0]

        #Restart the recording thread
        self.daemonize()

    #Reset the recorded times to a new time origin
    def resetClock(self):
        newTime = time.time()
        self.time -= (newTime - self.baseTime)
        self.baseTime = newTime

    #Set parameter values
    def setDensity(self,den):
        self.density = den

    def setSamplerate(self,rat):
        self.samplerate = rat

    #Read recorded values and reset the variables doing it
    def readValues(self):
        #Make sure all of the data lines up
        t = self.time
        tempLength = len(t)-1
        p = self.press[:tempLength]
        T = self.temp[:tempLength]
        d = self.depth[:tempLength]
        
        #Only keep the data that will not have been reported yet
        tempLength += 1
        self.time = self.time[tempLength:]
        self.press = self.press[tempLength:]
        self.temp = self.temp[tempLength:]
        self.depth = self.depth[tempLength:]
        
        return (t,p,T,d)









        
        
