"""
Created on 07 apr 2016

@author: Filip Lindau
"""
import sys
import PyTango
import threading
import logging
import time
import numpy as np
import Queue
from HalcyonTracking import HalcyonCommand, HalcyonTracking, PicomotorFollowParameters

logging.basicConfig(format='%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s', level=logging.INFO)


# ==================================================================
#   HalcyonDS Class Description:
#
#         Control of a KMLabs Halcyon
#
# ==================================================================
#     Device States Description:
#
#   DevState.ON :       Connected to Halcyon driver
#   DevState.OFF :      Disconnected from Halcyon
#   DevState.FAULT :    Error detected
#   DevState.UNKNOWN :  Communication problem
#   DevState.MOVING :  Motor moving
#   DevState.INIT :     Initializing Halcyon driver.
# ==================================================================


class HalcyonDS(PyTango.Device_4Impl):

    # ------------------------------------------------------------------
    #     Device constructor
    # ------------------------------------------------------------------
    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        HalcyonDS.init_device(self)

# ------------------------------------------------------------------
#     Device destructor
# ------------------------------------------------------------------
    def delete_device(self):
        with self.streamLock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stopThread()


# ------------------------------------------------------------------
#     Device initialization
# ------------------------------------------------------------------
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

        except Exception:
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
        while self.stopStateThreadFlag is False:
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
        has been established. Here all devices are initialized.
        """
        with self.streamLock:
            self.info_stream('Entering unknownHandler')
        connectionTimeout = 1.0
        self.set_status('Connecting to frequency counter, labjack, and picomotor')

        # Need to connect to frequency counter, ad7991,
        # and picomotor

        while self.stopStateThreadFlag is False:
            # AD7991:
            try:
                self.ad7991Device = PyTango.DeviceProxy(self.ad7991DeviceName)
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create ad7991 device ', str(self.ad7991DeviceName))))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=int(connectionTimeout))
                self.set_status('Could not create ad7991 device')

            # Frequency counter:
            try:
                self.frequencyDevice = PyTango.DeviceProxy(self.frequencyDeviceName)
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create Frequency counter device ', str(self.frequencyDeviceName))))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=int(connectionTimeout))
                self.set_status('Could not create frequency counter device')

            # Picomotor
            try:
                self.picomotorDevice = PyTango.DeviceProxy(self.picomotorDeviceName)
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create picomotor device ', str(self.picomotorDeviceName))))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=int(connectionTimeout))
                self.set_status('Could not create picomotor device')

            # RedPitaya
            try:
                self.redpitayaDevice = PyTango.DeviceProxy(self.redpitayaDeviceName)
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Could not create redpitaya device ', str(self.redpitayaDeviceName))))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=int(connectionTimeout))
                self.set_status('Could not create redpitaya device')
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

        apiutil = PyTango.ApiUtil.instance()
        if apiutil.get_asynch_cb_sub_model() != PyTango.cb_sub_model.PUSH_CALLBACK:
            apiutil.set_asynch_cb_sub_model(PyTango.cb_sub_model.PUSH_CALLBACK)

        while self.stopStateThreadFlag is False:
            retries += 1
            if retries > maxTries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break

            # Init parameters and variables
            self.errorFrequency = 0
            self.waitingForFrequency = False
            self.piezoVoltage = 0
            self.waitingForVoltage = False
            self.modelock = False
            self.waitingForModelock = False
            self.picomotorPosition = 0
            self.waitingForPicomotor = False
            self.errorTrace = np.zeros(4000)
            self.waitingForRedpitaya = False
            self.jitter = 0.0
            self.sampleTime = 1 / 1.5625e7

            # Init frequency counter by testing read
            try:
                with self.streamLock:
                    self.info_stream('Reading frequency')
                with self.attrLock:
                    self.errorFrequency = self.frequencyDevice.read_attribute('frequency')
            except Exception, ex:
                self.error_stream('Error reading error frequency')
                self.error_stream(''.join((ex)))
                self.checkCommands(blockTime=int(waitTime))

            # Init ad7991
            try:
                with self.streamLock:
                    self.info_stream('Initializing AD7991')
                with self.attrLock:
                    self.ad7991Device.write_attribute('voltagereference', 'vcc')
                    self.piezoVoltage = self.ad7991Device.read_attribute(''.join(('channel', str(self.ad7991PiezoChannel))))
                    modelockVoltage = self.ad7991Device.read_attribute(''.join(('channel', str(self.ad7991ModelockChannel))))
                    if modelockVoltage > 2.0:
                        self.modelock = True
                    else:
                        self.modelock = False
            except Exception, ex:
                self.error_stream('Error reading error frequency')
                self.error_stream(''.join((ex)))
                self.checkCommands(blockTime=int(waitTime))

            # Init picomotor (could write speed here)
            try:
                with self.streamLock:
                    self.info_stream('Reading picomotor position')
                with self.attrLock:
                    self.picomotorPosition = self.picomotorDevice.read_attribute('MotorPosition0A1')
            except Exception, ex:
                self.error_stream('Error reading picomotor position')
                self.error_stream(''.join((ex)))
                self.checkCommands(blockTime=int(waitTime))

            # Init redpitaya (write record length and samplerate)
            try:
                with self.streamLock:
                    self.info_stream('Setting up redpitaya')
                with self.attrLock:
                    self.redpitayaDevice.command_inout('stop')
                    self.redpitayaDevice.write_attribute('samplerate', 1 / self.sampleTime)
                    self.redpitayaDevice.write_attribute('recordlength', 4000)
                    self.redpitayaDevice.command_inout('start')
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Error when initializing device')))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=int(waitTime))

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
        while self.stopStateThreadFlag is False:
            self.info_stream('onhandler loop')
            with self.attrLock:
                state = self.get_state()
            if state not in handledStates:
                break
            # Read piezo voltage and modelock status from ad7991
            with self.attrLock:
                self.info_stream('Checking devices')
                if self.commandQueue.empty() == True:
                    try:
                        self.info_stream('ad7991 check')
                        if self.waitingForVoltage is False:
                            self.ad7991Device.read_attributes_asynch(''.join(('channel', str(self.ad7991PiezoChannel))),
                                                                     self.updatePiezovoltage)
                            self.waitingForVoltage = True
                        if self.waitingForModelock is False:
                            self.ad7991Device.read_attributes_asynch(''.join(('channel', str(self.ad7991ModelockChannel))),
                                                                     self.updatePiezovoltage)
                            self.waitingForModelock = True
                    except Exception, ex:
                        with self.streamLock:
                            self.error_stream(''.join(('Error reading ad7991: ', str(ex))))
                            self.set_state(PyTango.DevState.FAULT)
                        with self.attrLock:
                            self.modelock = None
                            self.piezoVoltage = None

            # Read error frequency from frequency counter
            with self.attrLock:
                try:
                    self.info_stream('error frequency check')
                    if self.waitingForFrequency is False:
                        self.frequencyDevice.read_attribute_asynch('frequency', self.updateFrequency)
                        self.waitingForFrequency = True
                except Exception, ex:
                    with self.streamLock:
                        self.error_stream(''.join(('Error reading frequency counter: ', str(ex))))
                        self.set_state(PyTango.DevState.FAULT)
                        self.errorFrequency = None

            # Read picomotor position from picomotor
            with self.attrLock:
                try:
                    self.info_stream('picomotor check')
                    if self.waitingForPicomotor is False:
                        self.picomotorDevice.read_attribute_asynch('MotorPosition0A1', self.updatePicomotor)
                        self.waitingForPicomotor = True
                except Exception, ex:
                    with self.streamLock:
                        self.error_stream('Error reading picomotor position')
                        self.error_stream(str(ex))
                    self.set_state(PyTango.DevState.FAULT)
                    with self.attrLock:
                        self.picomotorPosition = None

            # Read redpitaya:
            with self.attrLock:
                try:
                    self.info_stream('picomotor check')
                    if self.waitingForPicomotor is False:
                        self.picomotorDevice.read_attribute_asynch('MotorPosition0A1', self.updatePicomotor)
                        self.waitingForPicomotor = True
                        if self.redpitayaChannel == 1:
                            self.redpitayaDevice.read_attribute_asynch('waveform1', self.updateRedpitaya)
                        else:
                            self.redpitayaDevice.read_attribute_asynch('waveform2', self.updateRedpitaya)
                except Exception, ex:
                    with self.streamLock:
                        self.error_stream('Error reading redpitaya')
                        self.error_stream(str(ex))
                    self.set_state(PyTango.DevState.FAULT)
                    self.errorTrace = None
                    self.jitter = None

            with self.attrLock:
                # Update parameters for the tracking state machine
                self.halcyonTracking.modelock = self.modelock
                self.halcyonTracking.jitter = self.jitter
                self.halcyonTracking.errorFrequency = self.errorFrequency
                self.halcyonTracking.piezoVoltage = self.piezoVoltage
                # Update the state machine
                self.halcyonTracking.stateHandlerDispatcher()
                self.checkCommands()    # Check commands - there should be one from halcyonTracking

            self.checkCommands(blockTime=int(waitTime))

    def faultHandler(self, prevState):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.streamLock:
            self.info_stream('Entering faultHandler')
        handledStates = [PyTango.DevState.FAULT]
        waitTime = 0.1
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag is False:
            if self.get_state() not in handledStates:
                break

            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > maxTries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.checkCommands(blockTime=int(waitTime))

    def offHandler(self, prevState):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.streamLock:
            self.info_stream('Entering offHandler')
        self.set_state(PyTango.DevState.ON)

    def picomotorFollowFunction(self):
        # with self.streamLock:
        #     self.info_stream('Entering picomotorFollowFunction')

        #
        # Check conditions for moving picomotor (modelocked, frequency locked,
        # outside central piezo range, picomotor not recently moved):
        #
        moveMotorCondition = True
        with self.attrLock:
            if self.modelock is False:
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
        if moveMotorCondition is True:
            if self.picomotorFollowParameters.followState == 'idle':
                self.picomotorFollowParameters.followState = 'adjusting'
            with self.attrLock:
                deltaV = 50 - self.piezoVoltage
                newPos = self.picomotorPosition - deltaV
                if self.piezoVoltage < 50:
                    deltaV = 5
                else:
                    deltaV = -5
                deltaV
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
                # with self.streamLock:
                # self.debug_stream('checkCommands: blockTime == 0')
                cmd = self.commandQueue.get(block=False)
            else:
                # with self.streamLock:
                # self.debug_stream('checkCommands: blockTime != 0')
                cmd = self.commandQueue.get(block=True, timeout=blockTime)
#             with self.streamLock:
#                 self.info_stream(str(cmd.command))
            if cmd.command == 'writeMotorPosition':
                newPos = cmd.data
                with self.streamLock:
                    self.info_stream(''.join(('Write motor position ', str(newPos))))
                with self.attrLock:
                    self.picomotorDevice.write_attribute('motorposition0a1', newPos, wait=False)

            elif cmd.command == 'writeMotorPositionRelative':
                steps = cmd.data
                with self.streamLock:
                    self.info_stream(''.join(('Checkcommands: Moving picomotor ', str(steps), ' steps')))
                with self.attrLock:
                    self.picomotorDevice.write_attribute('CurrentMotor', 0, wait=False)
                    self.picomotorDevice.command_inout('MoveCurrentMotorRelative', steps, wait=False)

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

            elif cmd.command == 'adjust':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.MOVING)

        except Queue.Empty:
            # with self.streamLock:
            #     self.debug_stream('checkCommands: queue empty')

            pass

    def updateFrequency(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.errorFrequency = attr.value
            else:
                self.errorFrequency = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForFrequency = False

    def updatePiezovoltage(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.piezoVoltage = attr.value * 10
            else:
                self.piezoVoltage = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForVoltage = False

    def updateModelock(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                if attr.value > 2.0:
                    self.modelock = True
                else:
                    self.modelock = False
            else:
                self.modelock = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForModelock = False

    def updatePicomotor(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.picomotorPosition = attr.value
            else:
                self.picomotorPosition = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForPicomotor = False

    def updatePicomotorWrite(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.picomotorPosition = attr.value
            else:
                self.picomotorPosition = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForPicomotor = False

    def updateRedpitaya(self, arg):
        attr = arg.argout[0]
        with self.attrLock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.errorTrace = attr.value
                # Jitter calculation according to kmlabs white paper
                fref = 2.9985e9
                Vpp = 0.8
                self.jitter = 1 / (2 * np.pi * fref) * self.errorTrace.std() / (Vpp / 2)

            else:
                self.errorTrace = None
                self.jitter = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForRedpitaya = False


# ------------------------------------------------------------------
#     Always excuted hook method
# ------------------------------------------------------------------
    def always_executed_hook(self):
        pass


# ------------------------------------------------------------------
#     ErrorFrequency attribute
# ------------------------------------------------------------------
    def read_ErrorFrequency(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading error frequency')))
        with self.attrLock:
            attr_read = self.errorFrequency
            if attr_read is None:
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

# ------------------------------------------------------------------
#     PicomotorPosition attribute
# ------------------------------------------------------------------
    def read_PicomotorPosition(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attrLock:
            attr_read = self.picomotorPosition
            if attr_read is None:
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

# ------------------------------------------------------------------
#     PicomotorFollow attribute
# ------------------------------------------------------------------
    def read_PicomotorFollow(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor follow')))
        with self.attrLock:
            attr_read = self.halcyonTracking.follow
            if attr_read is None:
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

    def is_PicomotorFollow_allowed(self, req_type):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


# ------------------------------------------------------------------
#     Modelocked attribute
# ------------------------------------------------------------------
    def read_Modelocked(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attrLock:
            attr_read = self.modelock
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            elif attr_read is False:
                attr.set_quality(PyTango.AttrQuality.ATTR_ALARM)
            attr.set_value(attr_read)

    def is_Modelocked_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     PiezoVoltage attribute
# ------------------------------------------------------------------
    def read_PiezoVoltage(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading piezo voltage')))
        with self.attrLock:
            attr_read = self.piezoVoltage
            if attr_read is None:
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

# ------------------------------------------------------------------
#     Jitter attribute
# ------------------------------------------------------------------
    def read_Jitter(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading jitter')))
        with self.attrLock:
            self.info_stream(''.join(('Jitter: ', str(self.jitter))))
            attr_read = self.jitter
            q = PyTango.AttrQuality.ATTR_VALID
            if attr_read is None:
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

# ------------------------------------------------------------------
#     SampleTime attribute
# ------------------------------------------------------------------
    def read_SampleTime(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading sample time')))
        with self.attrLock:
            attr_read = self.sampleTime
            if attr_read is None:
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

# ------------------------------------------------------------------
#     ErrorTrace attribute
# ------------------------------------------------------------------
    def read_ErrorTrace(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading ErrorTrace')))
        with self.attrLock:
            attr_read = np.copy(self.errorTrace)
            with self.streamLock:
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read')))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read type ', str(type(attr_read)))))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read shape ', str(attr_read.shape))))
            if attr_read is None:
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


# ==================================================================
#
#     HalcyonDS command methods
#
# ==================================================================

# ------------------------------------------------------------------
#     On command:
#
#     Description: Start Halcyon driver
#
# ------------------------------------------------------------------
    def On(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmdMsg = HalcyonCommand('on')
        self.commandQueue.put(cmdMsg)

# ---- On command State Machine -----------------
    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     Stop command:
#
#     Description: Stop movement of all motors
#
# ------------------------------------------------------------------
    def Stop(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::Stop")))
        cmdMsg = HalcyonCommand('stop')
        self.commandQueue.put(cmdMsg)

# ---- Stop command State Machine -----------------
    def is_Stop_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


# ==================================================================
#
#     HalcyonDSClass class definition
#
# ==================================================================
class HalcyonDSClass(PyTango.DeviceClass):

    #     Class Properties
    class_property_list = {
        }

    #     Device Properties
    device_property_list = {
        'frequencyDeviceName':
            [PyTango.DevString,
             "Device name for the SuperLogics8080 frequency counter DS",
             []],
        'ad7991DeviceName':
            [PyTango.DevLong,
             "Device name for the ad7991 DS",
             []],
        'ad7991PiezoChannel':
            [PyTango.DevShort,
             "AD7991 channel connected to the piezo voltage",
             []],
        'ad7991ModelockChannel':
            [PyTango.DevShort,
             "AD7991 channel connected to the modelock signal",
             []],
        'picomotorDeviceName':
            [PyTango.DevString,
             "Device name for the picomotor DS",
             []],
        'redpitayaDeviceName':
            [PyTango.DevString,
             "Device name for the redpitaya DS",
             []],
        'redpitayaChannel':
            [PyTango.DevShort,
             "Redpitaya channel (1 or 2) that the error frequency is connected to",
             []],
        }

    #     Command definitions
    cmd_list = {
        'On':
            [[PyTango.DevVoid, ""],
             [PyTango.DevVoid, ""]],
        'Stop':
            [[PyTango.DevVoid, ""],
             [PyTango.DevVoid, ""]],
        }

    #     Attribute definitions
    attr_list = {
        'Modelocked':
            [[PyTango.DevBoolean,
              PyTango.SCALAR,
              PyTango.READ],
             {
                'description': "True if the oscillator is modelocked",
             }],
        'ErrorFrequency':
            [[PyTango.DevLong,
              PyTango.SCALAR,
              PyTango.READ],
             {
                'description': "Detected error frequency",
                'unit': 'Hz',
             }],
        'PicomotorPosition':
            [[PyTango.DevLong,
              PyTango.SCALAR,
              PyTango.READ_WRITE],
             {
                'description': "Picomotor step count",
                'unit': 'steps',
             }],
        'PicomotorFollow':
            [[PyTango.DevBoolean,
              PyTango.SCALAR,
              PyTango.READ_WRITE],
             {
                'description': "Picomotor moves to keep piezo voltage within limits if true",
                'unit': '',
                'Memorized': 'true',
             }],
        'PiezoVoltage':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ],
             {
                'description': "Piezo voltage level",
                'unit': '%'
             }],
        'Jitter':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ],
             {
                'description': "Jitter level from error trace",
                'unit': 's'
             }],
        'ErrorTrace':
            [[PyTango.DevDouble,
              PyTango.SPECTRUM,
              PyTango.READ, 6000],
             {
                'description': "Error signal as sampled by redpitaya",
                'unit': 'V'
             }],
        'SampleTime':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ],
             {
                'description': "Sample time for the error trace",
                'unit': 's'
             }],

        }


# ------------------------------------------------------------------
#     HalcyonDSClass Constructor
# ------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name)
        print "In HalcyonDSClass  constructor"

# ==================================================================
#
#     HalcyonDS class main method
#
# ==================================================================
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
