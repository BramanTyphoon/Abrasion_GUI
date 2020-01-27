# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget

from Ui_mainWindow import Ui_mainWindow
from rotor_driver import RotorDriver
from sensor_driver import SensorDriver
import threading
import time
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import argrelmax, argrelmin
import os
import csv

#Internal parameters
PLOT_UPDATE = 1
WRITE_FILE_UPDATE = 120 #How often to write to log and output files, in s
FEEDBACK_UPDATE = 10 #How often to run feedback control, in s
ESC_INIT_TIME = 2 #How long to allow the speed controllers to initialize, in s
PLOT_PREVIOUS = 20 #Time period (s) of previous readings to plot
DECOMP_LENGTH = 20 #Period of time (s) over which to decompose
                   #pressure data for system oscillation parameters
DEFAULT_PERIOD = 4.75 #Period to use if software is returning nonsense


class MainWindow(QWidget, Ui_mainWindow):
    """
    Class documentation goes here.
    """
    def __init__(self, parent=None):
        """
        Constructor
        
        :param parent: reference to the parent widget (QWidget)
        """
        super().__init__(parent)
        self.setupUi(self)

        self.baseTime = time.time() #Time origin from which to measure log events
        self.console.append("Abrasion GUI initialized at {}\n".format(time.ctime(self.baseTime)))
        self.logBody = "Abrasion GUI initialized at {}\n".format(time.ctime(self.baseTime)) #Text to output to the log file

        #Initiate the pressure/temperature sensor
        self.sd = SensorDriver(samplerate = float(self.pressureSampleEdit.text()),density=float(self.densityEdit.text()),baseTime=self.baseTime)
        self.sd.daemonize()
        #If the pressure/temperature sensor doesn't initialize, throw an error
        if self.sd.initialized:
            tempStr = "{:.2f}\t Pressure sensor successfully initialized\n".format(time.time()-self.baseTime)
            self.logBody += tempStr
            self.console.append(tempStr)
        else:
            tempStr = "{:.2f}\t ERROR: Pressure sensor not initialized\n".format(time.time()-self.baseTime)
            self.logBody += tempStr
            self.console.append(tempStr)
            self.writeLog()
            self.close()

        #Initiate a rotor driver for sin-wave fluctuations
        self.channels = [float(x) for x in self.initChannelsEdit.text().split(',')]
        self.flucrd = RotorDriver(channels = self.channels, magnitude = float(self.initPowerEdit.text())/100.,
                                  period = 4., runtime = 3600., comint = 1./float(self.sinSampleEdit.text()))
        tempStr = "{:.2f}\t Oscillatory rotor initialized\n".format(time.time()-self.baseTime)
        self.logBody += tempStr
        self.console.append(tempStr)

        #Initiate a rotor driver for period determination
        self.burstrd = RotorDriver(channels = self.channels, magnitude = 0,
                                   period = 0., runtime = ESC_INIT_TIME, comint = 1./float(self.sinSampleEdit.text()))
        tempStr = "{:.2f}\t Burst rotor initialized\n".format(time.time()-self.baseTime)
        self.logBody += tempStr
        self.console.append(tempStr)
        #self.burstrd.daemonize()
        #time.sleep(ESC_INIT_TIME) #Let speed controllers initialize
        tempStr = "{:.2f}\t Speed controllers initialized\n".format(time.time()-self.baseTime)
        self.logBody += tempStr
        self.console.append(tempStr)
        self.burstrd.setRuntime(float(self.initLengthEdit.text()))
        self.burstrd.setMagnitude(float(self.initBurstEdit.text())/100.)

        #Build the plot window
        self.axes = self.mainPlotFig.figure.add_subplot(111)
        self.axes.plot([0,1],[0,0])
        self.axes.xaxis.label = plt.xlabel("Time (hr)")
        self.axes.yaxis.label = plt.ylabel("Water level (m)")
        self.mainPlotFig.draw()
        

        #Import other defaults embedded in the GUI MainWindow
        self.outDir = "{}{}".format('//home//pi//output//',time.strftime("%Y%m%d%H%M%S//",time.localtime(self.baseTime))) #Output directory to which to save
        if not os.path.isdir(self.outDir):
            os.makedirs(self.outDir)
        self.density = float(self.densityEdit.text()) #Water density
        self.resToDuct = float(self.areaFactorEdit.text()) #Ratio of reservoir cross-section area to duct area
        self.feedbackAcc = float(self.fbackAccEdit.text()) #Feedback control loop 2nd derivative
        self.feedbackVel = float(self.fbackVelEdit.text()) #Feedback control loop 1st derivative
        self.pressureSample = 1./float(self.pressureSampleEdit.text()) #Sampling interval of pressure sensor, in seconds
        self.targetU = 0. #Target orbital velocity
        self.targetT = int(self.targetTEdit.text()) #Target multiple of system oscillation frequency
        self.systemT = 4.9 #Initial guess at system oscillation frequency
        self.targetTime = 3600. #How long the oscillatory run should last

        #Various parameters initialized
        self.tims = []
        self.pressures = []
        self.temperatures = []
        self.depths = []

        #Open output files
        self.initializeOutputFiles()

        #Finally, tack initial parameters onto log file
        self.logBody += self.formatParams()
        self.daemonize()

    def closeEvent(self, event):
        """
        When the app window closes, make sure to shut down the rotors and write
        final parameters to the log file and final data to the data output file.
        """
        # Shutdown the rotors
        self.flucrd.stop()
        self.burstrd.stop()
        # Write the final parameters to the log file
        with open("{}{}".format(self.outDir,'log.txt'),'w') as fil:
            fil.write("{}\n{}".format(self.logBody,self.formatParams()))
            fil.flush()
        out = self.sd.readValues()
        tims = out[0]
        pressures = out[1]
        depths = out[3]
        temperatures = out[2]
        #Write all remaining data to the output file
        with open("{}{}".format(self.outDir,'output.csv'),'a') as fil:
            writ = csv.writer(fil)
            writ.writerows(zip(*[tims,pressures,temperatures,depths]))
            fil.flush()
        
        
    #Start a thread to read sensor data, plot it, and run feedback control system
    def daemonize(self):
        self.plotthread = threading.Thread(target=self.plotRun,args=(),daemon=True)
        self.plotthread.start()
        self.logthread = threading.Thread(target=self.loggingRun,args=(),daemon=True)
        self.logthread.start()

    ## Methods to be run in threads in background
    #Method that controls plotting in the background
    def plotRun(self):
        while True:
            templength = min(len(self.sd.time),int(PLOT_PREVIOUS/self.pressureSample))
            self.tims = self.sd.time[-templength:]
            self.pressures = self.sd.press[-templength:]
            self.depths = self.sd.depth[-templength:]
            self.temperatures = self.sd.temp[-templength:]

            """
            Unfortunately, attempting to plot the data constantly during
            experiments causes a segmentation fault. I've traced the fault to
            these three lines and determined it's due to the Matplotlib backend
            to PyQt. There's not much I can do about it, so nothing is pltted
            for now. Fortunately, plotting the data is not necessary for running
            the experiments.
            """
            #self.axes.clear()
            #self.axes.plot(self.tims,self.depths)
            #self.mainPlotFig.draw()
            if templength > 1:
                self.tempDisplay.setText("{:.1f}".format(self.temperatures[-1]))
                self.timeElapsedDisplay.setText("{:.2f}".format(self.tims[-1]/3600.))
                self.realUDisplay.setText("{:.2f}".format(self.resToDuct * (self.depths[-1]-self.depths[-2])/(self.tims[-1]-self.tims[-2])))
                self.realHDisplay.setText("{:.2f}".format(self.depths[-1]))
            time.sleep(PLOT_UPDATE)

            #Once a rotor stops, change button text
            if (self.sustainedStartButton.text() == "Stop") & (not self.flucrd.running):
                self.sustainedStartButton.setText("Start")
                tempStr = "{:.2f}s\t Oscillatory run end".format(time.time()-self.baseTime)
                self.console.append(tempStr)
                self.logBody += (tempStr + "\r\n")

            #After the apparatus completes a burst of the rotor, use the data
            #collected to determine the system oscillation frequency of the
            #water in the oscillatory flow tunnel.
            if (self.periodStartButton.text() == "Stop") & (not self.burstrd.running):
                self.periodStartButton.setText("Start")
                tempStr = "{:.2f}s\t Burst run end".format(time.time()-self.baseTime)
                self.console.append(tempStr)
                self.logBody += (tempStr + "\r\n")
                time.sleep(DECOMP_LENGTH)
                tim = self.sd.time[-int((self.burstrd.runtime+DECOMP_LENGTH)/self.pressureSample):]
                self.decomposed = self.decomposeSignal(self.sd.depth[-int((self.burstrd.runtime+DECOMP_LENGTH)/self.pressureSample):],tim)
                #Sometimes the pressure sensor signal:noise ratio is too low
                #during this calibration exercise to perform accurate spectral
                #analysis, in which case the default frequency is used. The 
                #experimentally-determined oscillation period does not stray 
                #far from 4.75 s
                if np.isinf(self.decomposed[0]):
                    self.systemT = DEFAULT_PERIOD
                    tempStr = "{:.2f}s\t System oscillation period calculated: {:.2f} s".format(time.time()-self.baseTime,self.decomposed[0])
                    tempStr = "{:.2f}s\t System oscillation period invalid, using default: {:.2f} s".format(time.time()-self.baseTime,self.systemT)
                    self.console.append(tempStr)
                    self.logBody += (tempStr + "\r\n")
                    self.actualTDisplay.setText('{:.2f}'.format(self.systemT))
                    self.flucrd.setPeriod(self.targetT * self.systemT)
                else:
                    self.systemT = self.decomposed[0]
                    tempStr = "{:.2f}s\t System oscillation period calculated: {:.2f} s".format(time.time()-self.baseTime,self.systemT)
                    self.console.append(tempStr)
                    self.logBody += (tempStr + "\r\n")
                    self.actualTDisplay.setText('{:.2f}'.format(self.systemT))
                    self.flucrd.setPeriod(self.targetT * self.systemT)

    #Method that uses feedback controls to approach desired velocity, in the background
    def feedbackRun(self):
        while self.flucrd.running:
            time.sleep(FEEDBACK_UPDATE)
            #Keep track of the water level. If it goes below the pressure sensor,
            #stop running the experiment. The rotors need to remain submerged
            #to shed heat. Also, if we can't track water level, experimental
            #results are probably useless.
            if any([x for (x,y) in zip(self.sd.depth,self.sd.press) if ((x < -1)&(y > 0))]):
                self.flucrd.stop()
                tempStr = "{:.2f}s\t Depth exceeded min limit".format(time.time()-self.baseTime)
                self.console.append(tempStr)
                continue
            #Attempt to decompose the oscillation frequency
            if len(self.sd.depth) >= int(FEEDBACK_UPDATE/self.pressureSample):
                tim = self.sd.time[-int(FEEDBACK_UPDATE/self.pressureSample):]
                decomped = self.decomposeSignal(self.sd.depth[-int(FEEDBACK_UPDATE/self.pressureSample):],tim)
                if (np.isinf(decomped[0]) | (decomped[0] == 0)):
                    continue
                #Use the system oscillation frequency and the peak pressures
                #detected by the sensor to estimate RMS velocity assuming a 
                #sinusoidal oscillation.
                systemFreq = 2. * np.pi / (self.systemT * float(self.targetT))
                self.actualUDisplay.setText("{:3.2f}".format(systemFreq * decomped[1] / np.sqrt(2.) * self.resToDuct))
                targeth = self.targetU * np.sqrt(2.) / systemFreq / self.resToDuct   
                #Take into account edge case of 0 magnitude oscillation request
                if (targeth <= 0):
                    self.flucrd.setMagnitude(0)
                else:
                    #Allow oscillations to vary within 10% of the target, but
                    #after that use feedback controls to slowly bring back to
                    #the target oscillation water level height
                    if (abs(decomped[2]/decomped[3]) < 0.9) | (abs(decomped[2]/decomped[3]) > 1.1):
                        herror = (targeth - decomped[1])/targeth
                        self.flucrd.setMagnitude([min(self.flucrd.mag[0] +
                                                      self.flucrd.mag[0]*herror*self.feedbackVel,1),
                                                      #self.flucrd.mag[0]*(1-abs(decomped[2]/decomped[3]))*self.feedbackAcc,1),
                                                  min(self.flucrd.mag[1] + self.flucrd.mag[1]*herror*self.feedbackVel,1)])
                    else:
                        herror = (targeth - decomped[1])/targeth
                        self.flucrd.setMagnitude(np.fmin(self.flucrd.mag + self.flucrd.mag*herror*self.feedbackVel,[1,1]))
                    for m in range(0,len(self.flucrd.mag)-1,1):
                        if self.flucrd.mag[m] <= 0:
                            self.flucrd.mag[m] = min(self.feedbackVel * herror,1)
    
    #Method used to log parameters, parameter changes, and control execution                        
    def loggingRun(self):
        while True:
            time.sleep(WRITE_FILE_UPDATE)
            with open("{}{}".format(self.outDir,'log.txt'),'w') as fil:
                fil.write("{}\n{}".format(self.logBody,self.formatParams()))
                fil.flush()
            out = self.sd.readValues()
            tims = out[0]
            pressures = out[1]
            depths = out[3]
            temperatures = out[2]
            with open("{}{}".format(self.outDir,'output.csv'),'a') as fil:
                writ = csv.writer(fil)
                writ.writerows(zip(*[tims,pressures,temperatures,depths]))
                fil.flush()

    ## Methods governing GUI behavior
    @pyqtSlot()
    def on_outDirInput_returnPressed(self):
        """
        Change the output directory to that input by user.
        """
        self.outDir = self.outDirInput.text()
        self.outDir = "{}{}".format(self.outDir,time.strftime("%Y%m%d%H%M%S//",time.localtime(self.baseTime))) #Output directory to which to save
        if not os.path.isdir(self.outDir):
            os.mkdir(self.outDir)
        self.outputfile.close()
        self.logfile.close()
        self.initializeOutputFiles()
        
        tempStr = "{:.2f}s\t Output directory changed to {}".format(time.time()-self.baseTime,self.outDir)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_browseButton_released(self):
        """
        This button should open the usual dialogue to select an output directory
        using a file explorer, but we have yet to implement.
        """
        # TODO: not implemented yet
        raise NotImplementedError
    
    @pyqtSlot()
    def on_initBurstEdit_returnPressed(self):
        """
        Change the power of the burst used to determine system oscillation 
        period. The power is a percentage indicating power relative to the
        rotor's maximum.
        """
        self.burstrd.setMagnitude(float(self.initBurstEdit.text())/100.)
        tempStr = "{:.2f}s\t Burst power estimate changed to {:.2f}%".format(time.time()-self.baseTime,self.burstrd.mag[0]*100)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_initLengthEdit_returnPressed(self):
        """
        Change the length of the burst used to determine system oscillation period
        """
        self.burstrd.setRuntime(float(self.initLengthEdit.text()))
        tempStr = "{:.2f}s\t Initial burst length changed to {:.2f}s".format(time.time()-self.baseTime,float(self.initLengthEdit.text()))
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_initPowerEdit_returnPressed(self):
        """
        Change the initial power of the rotor-driven oscillations. The power 
        is a percentage indicating power relative to the rotor's maximum.
        """
        self.flucrd.setMagnitude(float(self.initPowerEdit.text())/100.)
        tempStr = "{:.2f}s\t Initial power estimate changed to {:.2f}%".format(time.time()-self.baseTime,self.flucrd.mag[0]*100)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_initChannelsEdit_returnPressed(self):
        """
        Set the RaspberryPi servo board ports to which the rotors are attached
        """
        channelInput = [float(x) for x in self.initChannelsEdit.text().split(',')]
        self.flucrd.setChannels(channelInput)
        self.burstrd.setChannels(channelInput)
        tempStr = "{:.2f}s\t Rotor PWM channels changed to {}".format(time.time()-self.baseTime,channelInput)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
        
    @pyqtSlot()
    def on_fbackVelEdit_returnPressed(self):
        """
        Change the velocity of the feedback control system.
        """
        self.feedbackVel = float(self.fbackVelEdit.text())
        tempStr = "{:.2f}s\t Feedback loop velocity changed to {:.2f}".format(time.time()-self.baseTime,self.feedbackVel)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_fbackAccEdit_returnPressed(self):
        """
        Change the acceleration of the feedback control system. N.B. we haven't
        actually used feedback acceleration, so this control is currently a
        placeholder only.
        """
        self.feedbackAcc = float(self.fbackAccEdit.text())
        tempStr = "{:.2f}s\t Feedback loop acceleration changed to {:.2f}".format(time.time()-self.baseTime,self.feedbackAcc)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_pressureSampleEdit_returnPressed(self):
        """
        Tell the pressure sensor how often to sample. You don't want to set this
        too high or the amount of data and buffers explode.
        """
        self.pressureSample = 1. / float(self.pressureSampleEdit.text())
        self.sd.setSamplerate(float(self.pressureSampleEdit.text()))
        tempStr = "{:.2f}s\t Pressure sensor sampling interval changed to {:.2f} Hz".format(time.time()-self.baseTime,float(self.pressureSampleEdit.text()))
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_sinSampleEdit_returnPressed(self):
        """
        Change the rate at which changes in rotor power are pushed to the rotor
        speed controllers. If this is set too high, the controllers could get
        angry. Too low and they may not maintain the sinusoidal oscillation.
        """
        self.flucrd.setComint(1./float(self.sinSampleEdit.text()))
        self.burstrd.setComint(1./float(self.sinSampleEdit.text()))
        tempStr = "{:.2f}s\t PWM update rate changed to {:.2f} Hz".format(time.time()-self.baseTime,float(self.sinSampleEdit.text()))
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_densityEdit_returnPressed(self):
        """
        Change the water density used to calculate depth from pressure
        """
        self.density = float(self.densityEdit.text())
        self.sd.setDensity(self.density)
        tempStr = "{:.2f}s\t Water density changed to {:.2f}kg/m^3".format(time.time()-self.baseTime,self.density)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_areaFactorEdit_returnPressed(self):
        """
        Set the ratio of the cross-sectional area of the reservoir above the
        pressure sensor to the cross-sectional area of the horizontal duct
        where we want to calculate the velocity
        """
        self.resToDuct = float(self.areaFactorEdit.text())
        tempStr = "{:.2f}s\t Reservoir-to-duct area ratio changed to {:.2f}".format(time.time()-self.baseTime,self.resToDuct)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_periodStartButton_released(self):
        """
        Start the burst run and system period determination upon pressing of
        that button.
        """
        if self.periodStartButton.text() == "Start":
            self.burstrd.daemonize()
            self.periodStartButton.setText("Stop")
            tempStr = "{:.2f}s\t Burst run Start".format(time.time()-self.baseTime)
            self.console.append(tempStr)
            self.logBody += (tempStr + "\r\n")
        else:
            self.burstrd.stop()
            self.periodStartButton.setText("Start")
            tempStr = "{:.2f}s\t Burst run Stopped by user.\nSystem period calculated may be incorrect.".format(time.time()-self.baseTime)
            self.console.append(tempStr)
            self.logBody += (tempStr + "\r\n")
            time.sleep(DECOMP_LENGTH)
            tim = self.sd.time[-int((self.burstrd.runtime+DECOMP_LENGTH)/self.pressureSample):]
            self.decomposed = self.decomposeSignal(self.sd.depth[-int((self.burstrd.runtime+DECOMP_LENGTH)/self.pressureSample):],tim)
            self.systemT = self.decomposed[0]
            tempStr = "{:.2f}s\t System oscillation period calculated: {:.2f} s".format(time.time()-self.baseTime,self.systemT)
            self.console.append(tempStr)
            self.logBody += (tempStr + "\r\n")
            self.actualTDisplay.setText('{:.2f}'.format(self.systemT))
            self.flucrd.setPeriod(self.targetT * self.systemT)
    
    @pyqtSlot()
    def on_targetTEdit_returnPressed(self):
        """
        Currently barely implemented functionality. User can specify a 
        """
        self.targetT = float(self.targetTEdit.text())
        self.flucrd.setMultiple(self.targetT)
        tempStr = "{:.2f}s\t Target T multiplier changed to {:.0f}x".format(time.time()-self.baseTime,self.targetT)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_sustainedStartButton_released(self):
        """
        Start the experiment sinusoidal oscillations when that button is pressed
        """
        if self.sustainedStartButton.text() == "Start":
            self.flucrd.daemonize()
            self.sustainedStartButton.setText("Stop")
            tempStr = "{:.2f}s\t Oscillatory run Start".format(time.time()-self.baseTime)
            self.console.append(tempStr)
            self.logBody += (tempStr + "\r\n")
            self.feedthread = threading.Thread(target=self.feedbackRun,args=(),daemon=True)
            self.feedthread.start()
        else:
            self.flucrd.stop()
            self.flucrd.setMagnitude(0.)
            self.flucrd.setMagnitude(float(self.initPowerEdit.text())/100.)
            self.sustainedStartButton.setText("Start")
            tempStr = "{:.2f}s\t Oscillatory run Stopped by user".format(time.time()-self.baseTime)
            self.console.append(tempStr)
            self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_lineEdit_returnPressed(self):
        """
        Allow user to change the target oscillation velocity, characterized
        as RMS velocity. N.B. the internal calculations of this assume
        hydrostatic pressure inaccurately, and actual RMS velocity will be
        LOWER than user input.
        """
        self.targetU = float(self.lineEdit.text())
        tempStr = "{:.2f}s\t Target U changed to {:.2f} m/s".format(time.time()-self.baseTime,self.targetU)
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")
    
    @pyqtSlot()
    def on_targetTimeEdit_returnPressed(self):
        """
        Slot documentation goes here.
        """
        self.flucrd.setRuntime(float(self.targetTimeEdit.text())*3600.)
        tempStr = "{:.2f}s\t Attempted run time changed to {} hr\r\n".format(time.time()-self.baseTime,self.targetTimeEdit.text())
        self.console.append(tempStr)
        self.logBody += (tempStr + "\r\n")

    ## Utility methods
    def decomposeSignal(self,sig,tims):
        #Make sure signal is of even length
        if (len(sig)%2 > 0):
            sig = sig[1:]

        # Use fft to decompose
        sigf = np.fft.fft(sig)
        signalInterval = np.mean([abs(y-x) for x,y in zip(tims[:-1],tims[1:])])
        faxis = np.fft.fftfreq(len(sig),d=signalInterval)

        #Find the maximum frequency and magnitude associated
        sigf = 2. * np.abs(sigf[:(len(sig)/2-1)]/len(sig))

        
        if ((not np.isinf(faxis[np.argmax(sigf)])) & (faxis[np.argmax(sigf)]>0)):
             peaks1 = argrelmax(np.array(sig)-np.median(sig),order = int(np.ceil(faxis[np.argmax(sigf)]/signalInterval/8.)))
             peaks2 = argrelmin(np.array(sig)-np.median(sig),order = int(np.ceil(faxis[np.argmax(sigf)]/signalInterval/8.)))
        else:
             peaks1 = argrelmax(np.array(sig)-np.median(sig),order = 1)
             peaks2 = argrelmin(np.array(sig)-np.median(sig),order = 1)
        peaks = np.concatenate((peaks1[0],peaks2[0]))             
             
        return((1./faxis[np.argmax(sigf)],
                np.median([abs(sig[x]) for x in peaks]),
                np.median([sig[x] for x in peaks1[0]]),
                np.median([sig[x] for x in peaks2[0]])))

    def initializeOutputFiles(self):
        with open("{}{}".format(self.outDir,'output.csv'),'w') as fil:
            fil.write("{},{},{},{}\n".format("Time (s)","Pressure (Pa)","Temperature (C)","Depth (m)"))
            fil.flush()
        with open("{}{}".format(self.outDir,'log.txt'),'w') as fil:
            fil.write("{}\n{}".format(self.logBody,self.formatParams()))
            fil.flush()

    def formatParams(self):
        return("""FINAL PARAMETERS
                  I/O params
                  Output directory: {}

                  ROTOR control params
                  Rotor channels [{}]
                  Initial oscillatory rotor magnitude: {} %
                  Current oscillatory rotor magnitude: [{:.2f},{:.2f}] %
                  Rotor communication sampling rate: {} Hz
                  Oscillatory rotor runtime: {:.3f} hr
                  Burst rotor magnitude: {}%
                  Burst rotor runtime: {} s

                  MODEL params
                  System oscillation period: {:.2f} s
                  Target period multiplier: {}x
                  Target orbital velocity: {:.2f} m/s
                  Reservoir-to-duct cross-section area ratio: {}
                  Pressure sample rate: {} Hz
                  Water density: {} kg/m^3

                  FEEDBACK control params
                  Feedback velocity: {}
                  Feedback acceleration: {}
                  """.format(self.outDir,self.initChannelsEdit.text(),
                             self.initPowerEdit.text(),self.flucrd.mag[0]*100.,self.flucrd.mag[1]*100.,
                             self.sinSampleEdit.text(),
                             self.flucrd.runtime/3600.,self.initBurstEdit.text(),
                             self.initLengthEdit.text(),self.systemT,self.targetTEdit.text(),
                             self.targetU,self.areaFactorEdit.text(),
                             self.pressureSampleEdit.text(),self.densityEdit.text(),
                             self.fbackVelEdit.text(),self.fbackAccEdit.text()))
