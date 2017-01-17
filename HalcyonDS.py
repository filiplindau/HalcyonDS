'''
Created on 30 jan 2015

@author: Filip Lindau
'''
import sys
import PyTango
import Superlogics8080_control as sc
import u12
import threading
import logging
import time
import numpy as np
from socket import gethostname
import Queue

logging.basicConfig(level=logging.INFO)

class HalcyonCommand:
    def __init__(self, command, data=None):
        self.command = command
        self.data = data

class HalcyonDriver:
    def __init__(self):
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.moving = [False, False, False]
        
class PicomotorFollowParameters(object):
    def __init__(self):
        self.follow = False
        self.maxPiezoVoltage = 70
        self.minPiezoVoltage = 30
        self.updateDeadtime = 2.0
        self.lastUpdateTime = time.time()
        self.followState = 'idle'
        self.moveSteps = 2
        self.maxNumberAdjusts = 5
        self.adjustNumber = 0

class HalcyonTracking(object):
    def __init__(self, cmdQueue):
        # Configuration parameters:
        self.follow = False
        self.search = False
        self.maxPiezoVoltage = 70
        self.minPiezoVoltage = 30
        self.updateDeadtime = 2.0
        self.moveSteps = 2
        self.maxNumberAdjusts = 5  # Maximum number of adjusts of picomotor before going back to idle
        self.maxNumberFailedAdjusts = 3  # Maxmimum number of adjusts that overflowed the maxNumberAdjusts before declaring the lock non-functional
        self.picomotorAverageTime = 60.0  # Collecting picomotor positions with this rolling window time
        
        self.q = cmdQueue
        
        # Current values
        self.lastUpdateTime = time.time()
        self.adjustNumber = 0
        self.failedAdjustNumber = 0
        self.errorFrequency = None
        self.jitter = None
        self.errorFrequencyHistory = []
        self.piezoVoltage = 0.0
        self.modelock = False
        self.picomotorHistory = [(self.lastUpdateTime, 0)]
        self.lockdown = False

        
        # State machine settings
        self.halcyonState = 'nomodelock'
        self.prevState = self.halcyonState
        self.stateHandlerDict = {'idle': self.idleHandler,
                        'nomodelock': self.noModelockHandler,
                        'nofreqlock': self.noFreqLockHandler,
                        'adjusting': self.adjustingHandler,
                        'searching': self.searchingHandler}

        
    def stateHandlerDispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        try:
            self.stateHandlerDict[self.halcyonState]()
            if self.halcyonState != self.prevState:
                self.prevState = self.halcyonState
        except KeyError:
            self.stateHandlerDict['nomodelock']()
            if self.halcyonState != self.prevState:
                self.prevState = self.halcyonState
        
    def idleHandler(self):
        logging.info('Entering idleHandler')
        if self.modelock == False:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency > 0:
            self.halcyonState = 'nofreqlock'
        elif self.piezoVoltage < self.minPiezoVoltage or self.piezoVoltage > self.maxPiezoVoltage:
            logging.info('Idle: Piezo in adjust range')
            if self.follow == True:
                if self.lockdown == True:
                    # If the device was in lockdown mode
                    # and then was turned back to follow (someone issued 
                    # wrote follow=True) we exit lockdown mode.
                    self.lockdown = False
                    cmdMsg = HalcyonCommand('on')
                    self.q.put(cmdMsg)
                logging.info('Following')
                self.adjustNumber = 0
                self.halcyonState = 'adjusting'
            else:
                logging.info('Not following')

    def noModelockHandler(self):
        logging.info('Entering noModeLockHandler')
        if self.modelock == True:
            self.halcyonState = 'nofreqlock'
            
    def noFreqLockHandler(self):
        logging.info('Entering noFreqLockHandler')
        if self.modelock == False:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency == 0:
            self.halcyonState = 'idle'
        elif self.search == True:
            self.halcyonState = 'searching'

    def lockdownHandler(self):
        # Too many adjustment attempts where made. Do not exit this state
        # until the oscillator drops out of modelock or state is forcibly changed
        # from the outside.
        # 
        # The reason for coming to this state is probably that the device has lost
        # either the 3 GHz RF signal or the photodiode signal.
        #
        logging.info('Entering lockdownHandler')
        if self.modelock == False:
            self.halcyonState = 'nomodelock'
            
    def adjustingHandler(self):
        logging.info('Entering adjustingHandler')
        logging.info(''.join(('Adjustment #', str(self.adjustNumber))))
        if self.modelock == False:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency > 0:
            self.halcyonState = 'nofreqlock'
        elif self.follow == True:
            if self.adjustNumber < self.maxNumberAdjusts:
                logging.info('Adjustment number ok')
                t = time.time()
                if t - self.lastUpdateTime > self.updateDeadtime:
                    logging.info('Adjustment time ok')
                    self.adjustNumber += 1
                    self.lastUpdateTime = t
                    # Only keep adjusting if we are outside a +- 5 range of center:
                    if np.abs(self.piezoVoltage - 50) > 5:
                        logging.info('Adjustment voltage in range')
                        if np.abs(self.piezoVoltage) > 1:
                            # If the piezo voltage is too low there is a problem,
                            # probably a missing signal
                            if self.piezoVoltage < 50:
                                cmdMsg = HalcyonCommand('writeMotorPositionRelative', -self.moveSteps)
                                self.picomotorHistory.append((t, self.picomotorHistory[-1][1] - self.moveSteps))
                            else:
                                cmdMsg = HalcyonCommand('writeMotorPositionRelative', self.moveSteps)
                                self.picomotorHistory.append((t, self.picomotorHistory[-1][1] + self.moveSteps))
                            self.q.put(cmdMsg)
                            if self.picomotorHistory[-1][0] - self.picomotorHistory[0][0] > self.picomotorAverageTime:
                                self.picomotorHistory.pop(0) 
                    else:
                        # In range. Reset adjustNumber and failedAdjustNumber
                        self.adjustNumber = 0
                        self.failedAdjustNumber = 0
                        self.halcyonState = 'idle'
            else:
                # Reached max number of adjusts
                self.adjustNumber = 0
                self.failedAdjustNumber += 1
                self.halcyonState = 'idle'  # Not enough attempts, have another go
                if self.failedAdjustNumber > self.maxNumberFailedAdjusts:
                            # Too many adjustment attempts where made. Entering lockdown. Do not exit 
                            # this mode until someone issues a writes True to follow attribute.
                            # 
                            # The reason for coming to this mode is probably that the device has lost
                            # either the 3 GHz RF signal or the photodiode signal.
                            #
                    self.follow = False
                    self.lockdown = True
                    self.failedAdjustNumber = 0
                    cmdMsg = HalcyonCommand('alarm')
                    self.q.put(cmdMsg)

        
    def searchingHandler(self):
        logging.info('Entering searchingHandler')
        if self.modelock == False:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency == 0:
            self.halcyonState = 'idle'



#==================================================================
#   HalcyonDS Class Description:
#
#         Control of a NewFocus (Newport) Halcyon
#
#==================================================================
#     Device States Description:
#
#   DevState.ON :       Connected to Halcyon driver
#   DevState.OFF :      Disconnected from Halcyon
#   DevState.FAULT :    Error detected
#   DevState.UNKNOWN :  Communication problem
#   DevState.MOVING :  Motor moving
#   DevState.INIT :     Initializing Halcyon driver.
#==================================================================


class HalcyonDS(PyTango.Device_4Impl):

#--------- Add you global variables here --------------------------

#------------------------------------------------------------------
#     Device constructor
#------------------------------------------------------------------
    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        HalcyonDS.init_device(self)

#------------------------------------------------------------------
#     Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        with self.streamLock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stopThread()


#------------------------------------------------------------------
#     Device initialization
#------------------------------------------------------------------
    def init_device(self):
        self.streamLock = threading.Lock()
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::init_device()")))
        self.set_state(PyTango.DevState.UNKNOWN)
        self.get_device_properties(self.get_device_class())

        # Try stopping the stateThread if it was started before. Will fail if this
        # is the initial start.
        try:
            self.stopThread()

        except Exception, e:
            pass
        
        try:
            self.picomotorFollowParameters
        except:
            self.picomotorFollowParameters = PicomotorFollowParameters()

        self.attrLock = threading.Lock()
        self.eventIdList = []
        self.stateThread = threading.Thread()
        threading.Thread.__init__(self.stateThread, target=self.stateHandlerDispatcher)

        self.commandQueue = Queue.Queue(100)
        self.labjackList = []
        
        self.faultMsg = ''
        self.faultConditionList = []
        
        try:
            self.halcyonTracking = HalcyonTracking(self.commandQueue)
            self.halcyonTracking.maxNumberAdjusts = 10
            self.halcyonTracking.updateDeadtime = 1
            self.halcyonTracking.minPiezoVoltage = 30
            self.halcyonTracking.maxPiezoVoltage = 70
        except:            
            self.halcyonTracking = HalcyonTracking(self.commandQueue)
    #                self.halcyonTracking.follow = False
            self.halcyonTracking.maxNumberAdjusts = 10
            self.halcyonTracking.updateDeadtime = 1
            self.halcyonTracking.minPiezoVoltage = 30
            self.halcyonTracking.maxPiezoVoltage = 70
        

        self.stateHandlerDict = {PyTango.DevState.ON: self.onHandler,
                                PyTango.DevState.MOVING: self.onHandler,
                                PyTango.DevState.ALARM: self.onHandler,
                                PyTango.DevState.FAULT: self.onHandler,
                                PyTango.DevState.INIT: self.initHandler,
                                PyTango.DevState.UNKNOWN: self.unknownHandler,
                                PyTango.DevState.OFF: self.offHandler}

        self.stopStateThreadFlag = False

        self.stateThread.start()


    def stateHandlerDispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        prevState = self.get_state()
        while self.stopStateThreadFlag == False:
            try:
                self.stateHandlerDict[self.get_state()](prevState)
                prevState = self.get_state()
            except KeyError:
                self.stateHandlerDict[PyTango.DevState.UNKNOWN](prevState)
                prevState = self.get_state()

    def stopThread(self):
        """Stops the state handler thread by setting the stopStateThreadFlag
        """
        self.stopStateThreadFlag = True
        self.stateThread.join(3)
        self.frequencyDevice.close()


    def unknownHandler(self, prevState):
        """Handles the UNKNOWN state, before communication with the hardware devices
        has been established. 
        """
        with self.streamLock:
            self.info_stream('Entering unknownHandler')
        connectionTimeout = 1.0
        self.set_status('Connecting to frequency counter, labjack, and picomotor')

        # Need to connect to frequency counter, labjack,
        # and picomotor

        while self.stopStateThreadFlag == False:
            # Labjack:
            try:
                with self.streamLock:
                    self.info_stream('Opening labjacks')
                self.labjackDevice = u12.U12()
                self.labjackList = []
                s = self.labjackDevice.listAll()['serialnumList']
                for sn in s:
                    if sn == 9999:
                        break
                    self.labjackList.append(sn)
                with self.streamLock:
                    self.info_stream(''.join(('Labjacks found: ', str(self.labjackList))))
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Could not list labjacks: ', str(e))))
                self.set_status('Could not list labjacks, trying again')
                self.checkCommands(blockTime=connectionTimeout)
                continue
            try:
                self.labjackIndex = self.labjackList.index(self.labjackSerial)
            except ValueError:
                with self.streamLock:
                    self.error_stream(''.join(('Labjack serial: ', str(self.labjackSerial), ' not in list ', str(self.labjackList))))
                self.set_status('Wrong labjack serial')
                continue
            
            # Frequency counter:
            try:
                with self.streamLock:
                    self.info_stream('Closing old frequency device')
                self.frequencyDevice.close()
            except:
                pass
            try:
                with self.streamLock:
                    self.info_stream(''.join(('Opening frequency device on port ', self.frequencyPort)))
                self.frequencyDevice = sc.Superlogics8080_control(self.frequencyPort)
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Could not connect to frequency counter ', self.frequencyPort)))
                with self.streamLock:
                    self.error_stream(str(e))
                self.set_status('Could not connect to frequency counter')
                self.checkCommands(blockTime=connectionTimeout)
                continue

            # Picomotor
            try:
                self.picomotorDevice = PyTango.DeviceProxy(self.picomotorDeviceName)
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create picomotor device ', str(self.picomotorDeviceName))))
                    self.error_stream(str(e))
                self.checkCommands(blockTime=connectionTimeout)
                self.set_status('Could not create picomotor device')
                continue

            # RedPitaya
            try:
                self.redpitayaDevice = PyTango.DeviceProxy(self.redpitayaDeviceName)
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create redpitaya device ', str(self.redpitayaDeviceName))))
                    self.error_stream(str(e))
                self.checkCommands(blockTime=connectionTimeout)
                self.set_status('Could not create redpitaya device')
                continue
            self.set_state(PyTango.DevState.INIT)
            break

    def initHandler(self, prevState):
        """Handles the INIT state. Query Halcyon device to see if it is alive.
        """
        with self.streamLock:
            self.info_stream('Entering initHandler')
        waitTime = 1.0
        self.set_status('Initializing devices')
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag == False:
            retries += 1
            if retries > maxTries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break
            try:
                with self.streamLock:
                    self.info_stream('Trying to connect...')
                self.errorFrequency = 0
                self.piezoVoltage = 0
                self.modelock = False
                self.picomotorPosition = 0
                self.errorTrace = np.zeros(4000)
                self.jitter = 0.0
                self.sampleTime = 1 / 1.5625e7

            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Error when initializing device')))
                    self.error_stream(str(e))
                self.checkCommands(blockTime=waitTime)
                continue
            
            try:
                with self.streamLock:
                    self.info_stream('Reading picomotor position')
                with self.attrLock:
                    self.picomotorPosition = self.picomotorDevice.read_attribute('MotorPosition0A1')
            except Exception, e:
                self.error_stream('Error reading picomotor position')
                self.error_stream(''.join((str(e))))
                self.checkCommands(blockTime=waitTime)

            try:
                with self.streamLock:
                    self.info_stream('Setting up redpitaya')
                with self.attrLock:
                    self.redpitayaDevice.command_inout('stop')
                    self.redpitayaDevice.write_attribute('samplerate', 1 / self.sampleTime)
                    self.redpitayaDevice.write_attribute('recordlength', 4000)
                    self.redpitayaDevice.command_inout('start')
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Error when initializing device')))
                    self.error_stream(str(e))
                self.checkCommands(blockTime=waitTime)
                continue

            self.set_state(PyTango.DevState.ON)
            break

    def onHandler(self, prevState):
        """Handles the ON state. Connected to the Halcyon driver.
        Waits in a loop checking commands.
        """
        with self.streamLock:
            self.info_stream('Entering onHandler')
        handledStates = [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.MOVING, PyTango.DevState.FAULT]
        waitTime = 0.1
        self.set_status('On')
        while self.stopStateThreadFlag == False:
            self.info_stream('onhandler loop')
            with self.attrLock:
                state = self.get_state()
            if state not in handledStates:
                break
            if self.faultConditionList != []:
                # If there is a fault condition, see if it can be cleared
                self.faultConditionCheck()
                
            # Read piezo voltage and modelock status from labjack
            with self.attrLock:
                self.info_stream('Checking devices')
                if self.commandQueue.empty() == True:
                    if 'labjack' not in self.faultConditionList:
                        # Try accessing labjack if it is not in fault 
                        try:
                            self.info_stream('labjack check')
                            data = self.labjackDevice.aiSample(2, [0, 1], self.labjackIndex)['voltages']
                            self.info_stream('labjack ok')
                            self.piezoVoltage = data[0] * 10
                            if data[1] > 3:
                                self.modelock = True
                            else:
                                self.modelock = False
                        except Exception, e:
                            with self.streamLock:
                                self.error_stream(''.join(('Error reading labjack: ', str(e))))
#                                 self.set_state(PyTango.DevState.FAULT)
#                                 self.modelock = None
#                                 self.piezoVoltage = None
#                                 self.faultConditionList.append('labjack')
                            
            # Read error frequency from frequency counter
            if 'errorfrequency' not in self.faultConditionList:
                with self.attrLock:                
                    try:
                        self.info_stream('error frequency check')
                        self.errorFrequency = self.frequencyDevice.getFrequency()
                        self.info_stream('error frequency ok')
                    except Exception, e:
                        with self.streamLock:
                            self.error_stream(''.join(('Error reading frequency counter: ', str(e))))
#                             self.set_state(PyTango.DevState.FAULT)
#                             self.errorFrequency = None
                            
            # Read picomotor position from picomotor (this might not be needed...)
            if 'picomotor' not in self.faultConditionList:
                with self.attrLock:
                    try:      
                        self.info_stream('picomotor check')              
                        data = self.picomotorDevice.read_attribute('MotorPosition0A1')
                        if data.quality == PyTango.AttrQuality.ATTR_INVALID:
                            self.set_state(PyTango.DevState.FAULT)
                            self.picomotorPosition = None
                        else:
                            self.picomotorPosition = data.value
                            self.info_stream('picomotor ok')
                    except Exception, e:
                        with self.streamLock:
                            self.error_stream('Error reading picomotor position')
                            self.error_stream(str(e))
#                         self.set_state(PyTango.DevState.FAULT)
#                         self.picomotorPosition = None
#             if self.picomotorFollowParameters.follow == True:
#                 self.picomotorFollowFunction()
            
            # Read redpitaya:
            if 'redpitaya' not in self.faultConditionList:
                with self.attrLock:
                    try:
                        if self.redpitayaChannel == 1:
                            data = self.redpitayaDevice.read_attribute('waveform1')
                        else:        
                            self.info_stream('Reading waveform 2')
                            data = self.redpitayaDevice.read_attribute('waveform2')
                        self.info_stream('redpitaya ok')
                        self.errorTrace = data.value
                        # Jitter calculation according to kmlabs white paper
                        fref = 2.9985e9
                        Vpp = 0.8                
                        self.jitter = 1 / (2 * np.pi * fref) * self.errorTrace.std() / (Vpp / 2) 
                    except Exception, e:
                        with self.streamLock:
                            self.error_stream('Error reading redpitaya')
                            self.error_stream(str(e))
#                         self.set_state(PyTango.DevState.FAULT)
#                         self.errorTrace = None
#                         self.jitter = None
            
            self.halcyonTracking.modelock = self.modelock
            self.halcyonTracking.jitter = self.jitter
            self.halcyonTracking.errorFrequency = self.errorFrequency
            self.halcyonTracking.piezoVoltage = self.piezoVoltage
            self.halcyonTracking.stateHandlerDispatcher()
                        
            self.checkCommands(blockTime=waitTime)


    def faultHandler(self, prevState):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.streamLock:
            self.info_stream('Entering faultHandler')
        handledStates = [PyTango.DevState.FAULT]
        waitTime = 0.1
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag == False:
            if self.get_state() not in handledStates:
                break
            # First test frequency counter:
            try:
                conf = self.frequencyDevice.getConfiguration()
                # Todo: check configuration and correct
                with self.streamLock:
                    self.info_stream(''.join(('Frequency counter configuration: ', str(conf))))
            except Exception, e:
                self.set_state(PyTango.DevState.UNKNOWN)
            # Next, test labjack:
            try:
                self.labjackDevice.aiSample(2, [0, 1], self.labjackIndex)
            except Exception, e:
                with self.streamLock:
                    self.error_stream(''.join(('Error labjack analog input: ', str(e), ' trying reset.')))
                try:
                    self.labjackDevice.reset(self.labjackIndex)
                    time.sleep(2)
                    self.set_state(PyTango.DevState.INIT)
                except Exception, e:
                    with self.streamLock:
                        self.error_stream(''.join(('Error resetting labjack: ', str(e))))
                    self.set_state(PyTango.DevState.UNKNOWN)
            # Now, test picomotor:
            try:
                with self.streamLock:
                    self.info_stream('Reading picomotor position')
                with self.attrLock:
                    self.picomotorPosition = self.picomotorDevice.read_attribute('MotorPosition0A1')
            except Exception, e:
                self.error_stream('Error reading picomotor position')
                self.error_stream(''.join((e)))
                self.checkCommands(blockTime=waitTime)
            
            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > maxTries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.checkCommands(blockTime=waitTime)

    def offHandler(self, prevState):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.streamLock:
            self.info_stream('Entering offHandler')
        self.set_state(PyTango.DevState.ON)

    def faultConditionCheck(self):
        ''' Check which device is in fault and try to correct it.
        Updates the self.faultMsg and uses self.faultConditionList to
        determine which device should be checked 
        '''
        faultMsgTmp = ''
        
        if 'labjack' in self.faultConditionList:
            # There was a problem with the labjack.
            # Try resetting it
            with self.streamLock:
                self.error_stream('In faultConditionCheck: Resetting labjack')
            with self.attrLock:
                self.labjackDevice.reset(self.labjackIndex)
            time.sleep(2)
            try:
                with self.attrLock:                
                    self.labjackDevice.aiSample(2, [0, 1], self.labjackIndex)
                    self.faultConditionList.pop('labjack')
            except Exception:                
                faultMsgTmp = ''.join((faultMsgTmp, 'Labjack fault\n'))
        if 'errorfrequency' in self.faultConditionList:
            # There was a problem with the error frequency counter
            # Try closing it
            with self.attrLock:
                self.frequencyDevice.close()
            time.sleep(1)
            try:
                with self.attrLock:
                    self.frequencyDevice.getConfiguration()
                    self.faultConditionList.pop('errorfrequency')
            except:
                faultMsgTmp = ''.join((faultMsgTmp, 'Error frequency counter fault\n'))
        if 'picomotor' in self.faultConditionList:
            # There was a problem with the picomotor
            # Try initilizing it
            with self.attrLock:
                self.picomotorDevice.command_inout('init')
            time.sleep(1)
            try:
                with self.attrLock:
                    if self.picomotorDevice.state() == PyTango.DevState.ON:
                        self.faultConditionList.pop('picomotor')
                    else:
                        faultMsgTmp = ''.join((faultMsgTmp, 'Picomotor fault\n'))
            except Exception:
                faultMsgTmp = ''.join((faultMsgTmp, 'Picomotor fault\n'))
        if 'redpitaya' in self.faultConditionList:
            # There was a problem with the redpitaya
            # Try initializing it
            with self.attrLock:
                self.redpitayaDevice.command_inout('init')
            time.sleep(1)
            try:
                with self.attrLock:
                    if self.redpitayaDevice.state() == PyTango.DevState.ON:
                        self.faultConditionList.pop('redpitaya')
                    else:
                        faultMsgTmp = ''.join((faultMsgTmp, 'Redpitaya fault\n'))
            except Exception:
                faultMsgTmp = ''.join((faultMsgTmp, 'Redpitaya fault\n'))
        
        with self.attrLock:
            self.faultMsg = faultMsgTmp
            self.set_status(self.faultMsg)
            if self.faultConditionList == []:
                self.set_state(PyTango.DevState.ON)
            else:
                self.set_state(PyTango.DevState.FAULT)
            

    def picomotorFollowFunction(self):
#        with self.streamLock:
#            self.info_stream('Entering picomotorFollowFunction')

        #
        # Check conditions for moving picomotor (modelocked, frequency locked,
        # outside central piezo range, picomotor not recently moved):
        #
        moveMotorCondition = True
        with self.attrLock:
            if self.modelock == False:
                moveMotorCondition = False
#                self.info_stream('Not modelocked')
            elif self.errorFrequency != 0:
                moveMotorCondition = False
#                self.info_stream('Not frequency locked')
            elif self.piezoVoltage > self.picomotorFollowParameters.minPiezoVoltage and self.piezoVoltage < self.picomotorFollowParameters.maxPiezoVoltage:
                moveMotorCondition = False
#                self.info_stream('Not out of piezo voltage range')
            elif time.time() - self.picomotorFollowParameters.lastUpdateTime < self.picomotorFollowParameters.updateDeadtime:
                moveMotorCondition = False
#                self.info_stream('Dead time not reached')
        if moveMotorCondition == True:
            if self.picomotorFollowParameters.followState == 'idle':
                self.picomotorFollowParameters.followState = 'adjusting'
            with self.attrLock:
                deltaV = 50 - self.piezoVoltage
                newPos = self.picomotorPosition - deltaV
                if self.piezoVoltage < 50:
                    deltaV = 5
                else:
                    deltaV = -5
            if self.picomotorFollowParameters.followState == 'adjusting':
                if self.picomotorFollowParameters.adjustNumber < self.picomotorFollowParameters.maxNumberAdjusts:
                    if self.piezoVoltage < 50:
                        cmdMsg = HalcyonCommand('writeMotorPositionRelative', -self.picomotorFollowParameters.moveSteps)
                    else:
                        cmdMsg = HalcyonCommand('writeMotorPositionRelative', self.picomotorFollowParameters.moveSteps)
                    self.commandQueue.put(cmdMsg)
                    self.picomotorFollowParameters.lastUpdateTime = time.time()
                    self.picomotorFollowParameters.adjustNumber += 1
                else:
                    self.picomotorFollowParameters.adjustNumber = 0
                    self.picomotorFollowParameters.followState = 'idle'


    def checkCommands(self, blockTime=0):
        """Checks the commandQueue for new commands. Must be called regularly.
        If the queue is empty the method exits immediately.
        """
#         with self.streamLock:
#             self.debug_stream('Entering checkCommands')
        try:
            if blockTime == 0:
#                 with self.streamLock:
#                     self.debug_stream('checkCommands: blockTime == 0')
                cmd = self.commandQueue.get(block=False)
            else:
#                 with self.streamLock:
#                     self.debug_stream('checkCommands: blockTime != 0')
                cmd = self.commandQueue.get(block=True, timeout=blockTime)
#             with self.streamLock:
#                 self.info_stream(str(cmd.command))
            if cmd.command == 'writeMotorPosition':
                newPos = cmd.data
                with self.streamLock:
                    self.info_stream(''.join(('Write motor position ', str(newPos))))
                with self.attrLock:
                    self.picomotorDevice.write_attribute('motorposition0a1', newPos)
            elif cmd.command == 'writeMotorPositionRelative':
                steps = cmd.data
                with self.streamLock:
                    self.info_stream(''.join(('Checkcommands: Moving picomotor ', str(steps), ' steps')))
                with self.attrLock:
                    self.picomotorDevice.write_attribute('CurrentMotor', 0)
                    self.picomotorDevice.command_inout('MoveCurrentMotorRelative', steps)
            elif cmd.command == 'stop' or cmd.command == 'standby':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    with self.attrLock:
                        self.picomotorDevice.command_inout('stop')

            elif cmd.command == 'off':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.OFF)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)
                    
            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ON)
                    

        except Queue.Empty:
#             with self.streamLock:
#                 self.debug_stream('checkCommands: queue empty')

            pass

#------------------------------------------------------------------
#     Always excuted hook method
#------------------------------------------------------------------
    def always_executed_hook(self):
        pass


#------------------------------------------------------------------
#     ErrorFrequency attribute
#------------------------------------------------------------------
    def read_ErrorFrequency(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading error frequency')))
        with self.attrLock:
            attr_read = self.errorFrequency
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_ErrorFrequency_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     PicomotorPosition attribute
#------------------------------------------------------------------
    def read_PicomotorPosition(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attrLock:
            attr_read = self.picomotorPosition
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_PicomotorPosition(self, attr):
        self.info_stream(''.join(('Writing picomotor position')))
        data = (attr.get_write_value())
        cmdMsg = HalcyonCommand('writeMotorPosition', data)
        self.commandQueue.put(cmdMsg)

    def is_PicomotorPosition_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     PicomotorFollow attribute
#------------------------------------------------------------------
    def read_PicomotorFollow(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor follow')))
        with self.attrLock:
#            attr_read = self.picomotorFollowParameters.follow
            attr_read = self.halcyonTracking.follow
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_PicomotorFollow(self, attr):
        self.info_stream(''.join(('Writing picomotor follow')))
        with self.attrLock:
            data = (attr.get_write_value())
#            self.picomotorFollowParameters.follow = data
            self.info_stream(''.join(('Setting halcyonTracking.follow to ', str(data))))
            self.halcyonTracking.follow = data
            self.set_state(PyTango.DevState.ON)

    def is_PicomotorFollow_allowed(self, req_type):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


#------------------------------------------------------------------
#     Modelocked attribute
#------------------------------------------------------------------
    def read_Modelocked(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attrLock:
            attr_read = self.modelock
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Modelocked_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     PiezoVoltage attribute
#------------------------------------------------------------------
    def read_PiezoVoltage(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading piezo voltage')))
        with self.attrLock:
            attr_read = self.piezoVoltage
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_PiezoVoltage_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     Jitter attribute
#------------------------------------------------------------------
    def read_Jitter(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading jitter')))
        with self.attrLock:
            self.info_stream(''.join(('Jitter: ', str(self.jitter))))
            attr_read = self.jitter
            q = PyTango.AttrQuality.ATTR_VALID
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = 0.0
            if self.errorFrequency > 0:
                self.info_stream('Jitter INVALID')
                q = PyTango.AttrQuality.ATTR_INVALID
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
#            attr.set_value(attr_read)
            attr.set_value_date_quality(attr_read, time.time(), q)

    def is_Jitter_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     SampleTime attribute
#------------------------------------------------------------------
    def read_SampleTime(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading sample time')))
        with self.attrLock:
            attr_read = self.sampleTime
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_SampleTime_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     ErrorTrace attribute
#------------------------------------------------------------------
    def read_ErrorTrace(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading ErrorTrace')))
        with self.attrLock:
            attr_read = np.copy(self.errorTrace)
            with self.streamLock:
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read')))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read type ', str(type(attr_read)))))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read shape ', str(attr_read.shape))))
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = np.array([0.0])
                with self.streamLock:
                    self.debug_stream(''.join(('In read_ErrorTrace: attr_read==None')))
            attr.set_value(attr_read, attr_read.shape[0])
        with self.streamLock:
            self.info_stream(''.join(('exit read_ErrorTrace')))

    def is_ErrorTrace_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


#==================================================================
#
#     HalcyonDS command methods
#
#==================================================================

#------------------------------------------------------------------
#     On command:
#
#     Description: Start Halcyon driver
#
#------------------------------------------------------------------
    def On(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmdMsg = HalcyonCommand('on')
        self.commandQueue.put(cmdMsg)

#---- On command State Machine -----------------
    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     Stop command:
#
#     Description: Stop movement of all motors
#
#------------------------------------------------------------------
    def Stop(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::Stop")))
        cmdMsg = HalcyonCommand('stop')
        self.commandQueue.put(cmdMsg)

#---- Stop command State Machine -----------------
    def is_Stop_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#------------------------------------------------------------------
#     ListLabjacks command:
#
#     Description: List the serial numbers of attached labjacks
#
#------------------------------------------------------------------
    def ListLabjacks(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::ListLabjacks")))
        with self.attrLock:
            return str(self.labjackList)

#---- Stop command State Machine -----------------
    def is_ListLabjacks_allowed(self):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

#==================================================================
#
#     HalcyonDSClass class definition
#
#==================================================================
class HalcyonDSClass(PyTango.DeviceClass):

    #     Class Properties
    class_property_list = {
        }


    #     Device Properties
    device_property_list = {
        'frequencyPort':
            [PyTango.DevString,
            "Com port of the 8080 frequency counter",
            [  ] ],
        'labjackSerial':
            [PyTango.DevLong,
            "Serial number of the labjack",
            [  ] ],
        'picomotorDeviceName':
            [PyTango.DevString,
            "Device name for the picomotor DS",
            [  ] ],
        'redpitayaDeviceName':
            [PyTango.DevString,
            "Device name for the redpitaya DS",
            [  ] ],
        'redpitayaChannel':
            [PyTango.DevShort,
            "Redpitaya channel (1 or 2) that the error frequency is connected to",
            [  ] ],
        }


    #     Command definitions
    cmd_list = {
        'On':
            [[PyTango.DevVoid, ""],
            [PyTango.DevVoid, ""]],
        'Stop':
            [[PyTango.DevVoid, ""],
            [PyTango.DevVoid, ""]],
        'ListLabjacks':
            [[PyTango.DevVoid, ""],
            [PyTango.DevString, ""]],
        }


    #     Attribute definitions
    attr_list = {
        'Modelocked':
            [[PyTango.DevBoolean,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"True if the oscillator is modelocked",
            } ],
        'ErrorFrequency':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"Detected error frequency",
                'unit':'Hz',
            } ],
        'PicomotorPosition':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ_WRITE],
            {
                'description':"Picomotor step count",
                'unit':'steps',
            } ],
        'PicomotorFollow':
            [[PyTango.DevBoolean,
            PyTango.SCALAR,
            PyTango.READ_WRITE],
            {
                'description':"Picomotor moves to keep piezo voltage within limits if true",
                'unit':'',
                'Memorized':'true',
            } ],
        'PiezoVoltage':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"Piezo voltage level",
                'unit':'%'
            } ],
        'Jitter':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"Jitter level from error trace",
                'unit':'s'
            } ],
        'ErrorTrace':
            [[PyTango.DevDouble,
            PyTango.SPECTRUM,
            PyTango.READ, 6000],
            {
                'description':"Error signal as sampled by redpitaya",
                'unit':'V'
            } ],
        'SampleTime':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description':"Sample time for the error trace",
                'unit':'s'
            } ],

        }


#------------------------------------------------------------------
#     HalcyonDSClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name);
        print "In HalcyonDSClass  constructor"

#==================================================================
#
#     HalcyonDS class main method
#
#==================================================================
if __name__ == '__main__':
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(HalcyonDSClass, HalcyonDS, 'HalcyonDS')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed, e:
        print '-------> Received a DevFailed exception:', e
    except Exception, e:
        print '-------> An unforeseen exception occured....', e
