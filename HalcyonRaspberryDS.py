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
from AttributeReadThreadClass import AttributeClass

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
        with self.stream_lock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stop_thread()


# ------------------------------------------------------------------
#     Device initialization
# ------------------------------------------------------------------
    def init_device(self):
        self.stream_lock = threading.Lock()
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::init_device()")))
        self.set_state(PyTango.DevState.UNKNOWN)
        self.get_device_properties(self.get_device_class())

        # Try stopping the stateThread if it was started before. Will fail if this
        # is the initial start.
        try:
            self.stop_thread()

        except Exception:
            pass

        try:
            self.picomotorFollowParameters
        except:
            self.picomotorFollowParameters = PicomotorFollowParameters()

        self.attributes = {}
        self.devices = {}

        self.attr_lock = threading.Lock()
        self.eventIdList = []
        self.state_thread = threading.Thread()
        threading.Thread.__init__(self.state_thread, target=self.stateHandlerDispatcher)

        self.command_queue = Queue.Queue(100)

        self.attributeTimeout = 3.0
        self.waitingForPicomotorWrite = False
        self.waitingForPicomotorId = None

        # Load up property values from the database:
        try:
            self.halcyon_tracking = HalcyonTracking(self.command_queue)
            self.halcyon_tracking.updateDeadtime = 1
            self.halcyon_tracking.minPiezoVoltage = self.piezoLowVoltage
            self.halcyon_tracking.maxPiezoVoltage = self.piezoHighVoltage
            self.halcyon_tracking.centerPiezoVoltage = (self.piezoHighVoltage + self.piezoLowVoltage) / 2.0
            self.info_stream('Using {0} piezoHighVoltage'.format(self.piezoHighVoltage))
        except:
            self.halcyon_tracking = HalcyonTracking(self.command_queue)
    #                self.halcyonTracking.follow = False
            self.halcyon_tracking.updateDeadtime = 1
            self.halcyon_tracking.minPiezoVoltage = 30
            self.halcyon_tracking.maxPiezoVoltage = 70
            self.halcyon_tracking.centerPiezoVoltage = (self.piezoHighVoltage + self.piezoLowVoltage) / 2.0

        try:
            self.halcyon_tracking.maxNumberAdjusts = self.maxNumberAdjusts
            self.info_stream('Using {0} maxNumberAdjusts'.format(self.maxNumberAdjusts))
        except:
            self.halcyon_tracking.maxNumberAdjusts = 30

        try:
            self.halcyon_tracking.piezoDeadband = self.piezoDeadBand
        except:
            self.halcyon_tracking.piezoDeadband = 10

        self.stateHandlerDict = {PyTango.DevState.ON: self.on_handler,
                                 PyTango.DevState.MOVING: self.on_handler,
                                 PyTango.DevState.ALARM: self.on_handler,
                                 PyTango.DevState.FAULT: self.on_handler,
                                 PyTango.DevState.INIT: self.init_handler,
                                 PyTango.DevState.UNKNOWN: self.unknown_handler,
                                 PyTango.DevState.OFF: self.off_handler}

        self.stop_state_thread_flag = False

        self.state_thread.start()

    def stateHandlerDispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        prevState = self.get_state()
        while self.stop_state_thread_flag is False:
            try:
                self.stateHandlerDict[self.get_state()](prevState)
                prevState = self.get_state()
            except KeyError:
                self.stateHandlerDict[PyTango.DevState.UNKNOWN](prevState)
                prevState = self.get_state()

    def stop_thread(self):
        """Stops the state handler thread by setting the stopStateThreadFlag
        """
        self.stop_state_thread_flag = True
        self.state_thread.join(3)
        try:
            for attr_c in self.attributes:
                self.info_stream(''.join(('Stopping thread ', str(attr_c.name))))
                attr_c.stop_read()
        except:
            pass

    def unknown_handler(self, prevState):
        """Handles the UNKNOWN state, before communication with the hardware devices
        has been established. Here all devices are initialized.
        """
        with self.stream_lock:
            self.info_stream('Entering unknownHandler')
        connection_timeout = 1.0
        self.set_status('Connecting to frequency counter, labjack, and picomotor')

        # Need to connect to frequency counter, ad7991, redpitaya
        # and picomotor

        while self.stop_state_thread_flag is False:
            # AD7991:
            try:
                self.devices['ad7991'] = PyTango.DeviceProxy(self.ad7991DeviceName)
            except Exception, ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Could not create ad7991 device ', str(self.ad7991DeviceName))))
                    self.error_stream(str(ex))
                self.check_commands(block_time=connection_timeout)
                self.set_status('Could not create ad7991 device')
            try:
                self.attributes['modelock'].stop_read()
            except KeyError:
                pass
            self.attributes['modelock'] = AttributeClass(''.join(('channel', str(self.ad7991ModelockChannel))),
                                                         self.devices['ad7991'], 0.3)
            try:
                self.attributes['piezo_voltage'].stop_read()
            except KeyError:
                pass
            self.attributes['piezo_voltage'] = AttributeClass(''.join(('channel', str(self.ad7991PiezoChannel))),
                                                              self.devices['ad7991'], 0.3)

            # Frequency counter:
            try:
                self.devices['frequency'] = PyTango.DeviceProxy(self.frequencyDeviceName)
            except Exception, ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Could not create Frequency counter device ',
                                               str(self.frequencyDeviceName))))
                    self.error_stream(str(ex))
                self.check_commands(block_time=connection_timeout)
                self.set_status('Could not create frequency counter device')
            try:
                self.attributes['error_frequency'].stop_read()
            except KeyError:
                pass
            self.attributes['error_frequency'] = AttributeClass('frequency', self.devices['frequency'], 0.3)

            # Picomotor
            try:
                self.devices['picomotor'] = PyTango.DeviceProxy(self.picomotorDeviceName)
            except Exception, ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Could not create picomotor device ', str(self.picomotorDeviceName))))
                    self.error_stream(str(ex))
                self.check_commands(block_time=connection_timeout)
                self.set_status('Could not create picomotor device')
            try:
                self.attributes['picomotor'].stop_read()
            except KeyError:
                pass
            self.attributes['picomotor'] = AttributeClass('MotorPosition0A1', self.devices['picomotor'], 0.3)

            # RedPitaya
            try:
                self.devices['redpitaya'] = PyTango.DeviceProxy(self.redpitayaDeviceName)
            except Exception, ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Could not create redpitaya device ', str(self.redpitayaDeviceName))))
                    self.error_stream(str(ex))
                self.check_commands(block_time=connection_timeout)
                self.set_status('Could not create redpitaya device')
            try:
                self.attributes['redpitaya'].stop_read()
            except KeyError:
                pass
            if self.redpitayaChannel == 1:
                self.attributes['redpitaya'] = AttributeClass('waveform1', self.devices['redpitaya'], 0.3)
            else:
                self.attributes['redpitaya'] = AttributeClass('waveform2', self.devices['redpitaya'], 0.3)

            self.set_state(PyTango.DevState.INIT)
            break

    def init_handler(self, prevState):
        """Handles the INIT state. Query Halcyon device to see if it is alive.
        """
        with self.stream_lock:
            self.info_stream('Entering init_handler')
        waitTime = 1.0
        self.set_status('Initializing devices')
        retries = 0
        maxTries = 5

        apiutil = PyTango.ApiUtil.instance()
        if apiutil.get_asynch_cb_sub_model() != PyTango.cb_sub_model.PUSH_CALLBACK:
            apiutil.set_asynch_cb_sub_model(PyTango.cb_sub_model.PUSH_CALLBACK)

        while self.stop_state_thread_flag is False:
            retries += 1
            if retries > maxTries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break

            # Init parameters and variables
            self.error_frequency = 0
            self.piezo_voltage = 0
            self.modelock = False
            self.picomotor_position = 0
            self.error_trace = np.zeros(4000)
            self.jitter = 0.0
            self.sample_time = 1 / 1.5625e7

            # Init frequency counter
            pass

            # Init ad7991
            try:
                with self.stream_lock:
                    self.info_stream('Initializing AD7991')
                with self.attr_lock:
                    self.devices['ad7991'].write_attribute('voltagereference', 'vcc')
            except Exception, ex:
                self.error_stream('Error writing ad7991 voltage reference')
                self.error_stream(''.join((ex)))
                self.append_status('Error writing ad7991 voltage reference', True)
                # noinspection PyTypeChecker
                self.check_commands(block_time=waitTime)

            # Init picomotor (could write speed here)
            pass

            # Init redpitaya (write record length and samplerate)
            try:
                with self.stream_lock:
                    self.info_stream('Setting up redpitaya')
                with self.attr_lock:
                    self.devices['redpitaya'].command_inout('stop')
                    self.devices['redpitaya'].write_attribute('samplerate', 1 / self.sample_time)
                    self.devices['redpitaya'].write_attribute('recordlength', 4000)
                    self.devices['redpitaya'].command_inout('start')
            except Exception, ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Error when initializing redpitaya')))
                    self.error_stream(str(ex))
                    self.append_status('Error when initializing redpitaya', True)
                self.check_commands(block_time=waitTime)

            self.halcyon_tracking.halcyonState = 'idle'
            self.set_state(PyTango.DevState.ON)
            break

    def on_handler(self, prevState):
        """Handles the ON state. Connected to the Halcyon driver.
        Waits in a loop checking commands.
        """
        with self.stream_lock:
            self.info_stream('Entering onHandler')
        handled_states = [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.MOVING]
        wait_time = 0.1
        self.set_status('On')
        while self.stop_state_thread_flag is False:
            self.info_stream('onhandler loop')
            with self.attr_lock:
                state = self.get_state()
            if state not in handled_states:
                break

            if self.command_queue.empty() is True:
                self.info_stream('Checking devices')
                # Modelock status
                if self.attributes['modelock'].read_event.is_set():
                        attr = self.attributes['modelock'].get_attribute()
                        if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                            with self.attr_lock:
                                if attr.value > 2.0:
                                    self.modelock = True
                                else:
                                    self.modelock = False
                        else:
                            self.error_stream('Error reading modelock')
                            self.set_state(PyTango.DevState.FAULT)
                            self.set_status('Error reading modelock')
                            self.modelock = None

                # Piezo voltage
                if self.attributes['piezo_voltage'].read_event.is_set():
                    attr = self.attributes['piezo_voltage'].get_attribute()
                    if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                        with self.attr_lock:
                            self.piezo_voltage = attr.value * 100
                    else:
                        self.error_stream('Error reading piezo voltage')
                        self.set_state(PyTango.DevState.FAULT)
                        self.set_status('Error reading piezo voltage')
                        self.piezo_voltage = None

                # Read error frequency from frequency counter
                if self.attributes['error_frequency'].read_event.is_set():
                    attr = self.attributes['error_frequency'].get_attribute()
                    if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                        with self.attr_lock:
                            self.error_frequency = attr.value
                    else:
                        self.error_stream('Error reading error frequency')
                        self.set_state(PyTango.DevState.FAULT)
                        self.set_status('Error reading error frequency')
                        self.error_frequency = None

                # Read picomotor position from picomotor
                if self.attributes['picomotor'].read_event.is_set():
                    attr = self.attributes['picomotor'].get_attribute()
                    if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                        with self.attr_lock:
                            self.picomotor_position = attr.value
                    else:
                        self.error_stream('Error reading picomotor')
                        self.set_state(PyTango.DevState.FAULT)
                        self.set_status('Error reading picomotor')
                        self.picomotor_position = None

                # Read redpitaya:
                if self.attributes['redpitaya'].read_event.is_set():
                    attr = self.attributes['redpitaya'].get_attribute()
                    if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                        with self.attr_lock:
                            self.error_trace = attr.value
                            fref = 2.9985e9
                            Vpp = 0.8
                            self.jitter = 1 / (2 * np.pi * fref) * self.error_trace.std() / (Vpp / 2)
                    else:
                        self.error_stream('Error reading redpitaya')
                        self.set_state(PyTango.DevState.FAULT)
                        self.set_status('Error reading redpitaya')
                        self.error_trace = None
                        self.jitter = None

            # If a picomotor write command has been issued, check if it is ready
            #self.update_picomotor_write()

            with self.attr_lock:
                # Update parameters for the tracking state machine:
                self.halcyon_tracking.modelock = self.modelock
                self.halcyon_tracking.jitter = self.jitter
                self.halcyon_tracking.errorFrequency = self.error_frequency
                self.halcyon_tracking.piezoVoltage = self.piezo_voltage
                # Update the state machine:
                self.halcyon_tracking.stateHandlerDispatcher()
            self.check_commands()    # Check commands - there should be one from halcyonTracking

            # with self.attr_lock:
            #     # Update state depending on lockdown and tracking parameters
            #     if self.halcyon_tracking.follow is True and self.halcyon_tracking.lockdown is True:
            #         self.set_state(PyTango.DevState.ON)

            # Check commands again, but with a short wait time
            # The queue is probably empty now
            self.check_commands(block_time=wait_time)

    def fault_handler(self, prevState):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.stream_lock:
            self.info_stream('Entering faultHandler')
        handled_states = [PyTango.DevState.FAULT]
        wait_time = 0.1
        retries = 0
        max_tries = 5

        # Will try to communicate with all devices
        while self.stop_state_thread_flag is False:
            if self.get_state() not in handled_states:
                break

            # Starting with picomotor
            with self.stream_lock:
                self.info_stream('Testing picomotor')
            with self.attr_lock:
                try:
                    st = self.picomotorDevice.state()
                    if st in [PyTango.DevState.ON, PyTango.DevState.OFF, PyTango.DevState.RUNNING, PyTango.DevState.MOVING, PyTango.DevState.ALARM, PyTango.DevState.STANDBY]:
                        self.set_state(PyTango.DevState.ON)
                    else:
                        self.info_stream(''.join(('Device state: ', str(st))))
                except Exception, ex:
                    self.error_stream(''.join(('Picomotor error, going to unknown state', str(ex))))
                    self.set_state(PyTango.DevState.UNKNOWN)
                    break

            # Testing redpitaya
            with self.stream_lock:
                self.info_stream('Testing redpitaya')
            with self.attr_lock:
                try:
                    st = self.redpitayaDevice.state()
                    if st in [PyTango.DevState.ON, PyTango.DevState.OFF, PyTango.DevState.RUNNING, PyTango.DevState.MOVING, PyTango.DevState.ALARM, PyTango.DevState.STANDBY]:
                        self.set_state(PyTango.DevState.ON)
                    else:
                        self.info_stream(''.join(('Device state: ', str(st))))
                except Exception, ex:
                    self.error_stream(''.join(('Redpitaya error, going to unknown state', str(ex))))
                    self.set_state(PyTango.DevState.UNKNOWN)
                    break

            # Testing ad7991
            with self.stream_lock:
                self.info_stream('Testing AD7991')
            with self.attr_lock:
                try:
                    st = self.ad7991Device.state()
                    if st in [PyTango.DevState.ON, PyTango.DevState.OFF, PyTango.DevState.RUNNING, PyTango.DevState.MOVING, PyTango.DevState.ALARM, PyTango.DevState.STANDBY]:
                        self.set_state(PyTango.DevState.ON)
                    else:
                        self.info_stream(''.join(('Device state: ', str(st))))
                except Exception, ex:
                    self.error_stream(''.join(('AD7991 error, going to unknown state', str(ex))))
                    self.set_state(PyTango.DevState.UNKNOWN)
                    break

            # Testing superlogics
            with self.stream_lock:
                self.info_stream('Testing superlogics')
            with self.attr_lock:
                try:
                    st = self.frequencyDevice.state()
                    if st in [PyTango.DevState.ON, PyTango.DevState.OFF, PyTango.DevState.RUNNING, PyTango.DevState.MOVING, PyTango.DevState.ALARM, PyTango.DevState.STANDBY]:
                        self.set_state(PyTango.DevState.ON)
                    else:
                        self.info_stream(''.join(('Device state: ', str(st))))
                except Exception, ex:
                    self.error_stream(''.join(('Supelogics error, going to unknown state', str(ex))))
                    self.set_state(PyTango.DevState.UNKNOWN)
                    break

            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > max_tries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.set_state(PyTango.DevState.FAULT)
            while self.get_state() == PyTango.DevState.FAULT:
                self.check_commands(block_time=wait_time)

    def off_handler(self, prevState):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.stream_lock:
            self.info_stream('Entering offHandler')
        self.set_state(PyTango.DevState.ON)

    def check_commands(self, block_time=0):
        """Checks the commandQueue for new commands. Must be called regularly.
        If the queue is empty the method exits immediately.
        """
        with self.stream_lock:
            self.debug_stream('Entering check_commands')
            self.debug_stream(''.join(('Queue length: ', str(self.command_queue.qsize()))))
            self.debug_stream(''.join(('block_time: ', str(block_time))))
        try:
            if block_time == 0:
                # with self.streamLock:
                # self.debug_stream('checkCommands: blockTime == 0')
                cmd = self.command_queue.get(block=False)
            else:
                # with self.streamLock:
                # self.debug_stream('checkCommands: blockTime != 0')
                cmd = self.command_queue.get(block=True, timeout=block_time)
            with self.stream_lock:
                self.info_stream(str(cmd.command))
            if cmd.command == 'writeMotorPosition':
                new_pos = cmd.data
                with self.stream_lock:
                    self.info_stream(''.join(('Write motor position ', str(new_pos))))
                with self.attr_lock:
                    self.info_stream('apa!')
                    self.attributes['picomotor'].attr_write(new_pos)
                with self.stream_lock:
                    self.info_stream(''.join(('Write motor position done')))

            elif cmd.command == 'writeMotorPositionRelative':
                steps = cmd.data
                t0 = time.time()
                with self.stream_lock:
                    self.info_stream(''.join(('Checkcommands: Moving picomotor ', str(steps), ' steps')))
                with self.attr_lock:
                    self.attributes['picomotor'].device_command('MoveCurrentMotorRelative', steps)

                with self.stream_lock:
                    self.debug_stream("Time spent writePicomotorRelative: {:.2f}s".format(time.time() - t0))
                    self.debug_stream("-----------------------------------------------------------------")

            elif cmd.command == 'stop' or cmd.command == 'standby':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.picomotorDevice.command_inout('stop')

            elif cmd.command == 'off':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.OFF)
                    self.set_status(cmd.data)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)

            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)
                    self.set_status(cmd.data)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ON)
                    self.set_status(cmd.data)

            elif cmd.command == 'adjust':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.MOVING)
                    self.set_status(cmd.data)

            self.command_queue.task_done()

        except Queue.Empty:
            # with self.streamLock:
            #     self.debug_stream('checkCommands: queue empty')

            pass

        self.debug_stream('Exit checkCommands')

    def updateFrequency(self, arg):
        attr = arg.argout[0]
        with self.attr_lock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.error_frequency = attr.value
                self.errorFrequencyLastUpdate = time.time()
            else:
                self.error_frequency = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForFrequency = False

    def updatePiezoVoltage(self, arg):
        attr = arg.argout[0]
        with self.attr_lock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.piezo_voltage = attr.value * 100
                self.piezoVoltageLastUpdate = time.time()
            else:
                self.piezo_voltage = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForVoltage = False
            self.info_stream("".join(("PiezoVoltage type: ", str(type(self.piezo_voltage)))))

    def updateModelock(self, arg):
        attr = arg.argout[0]
        with self.attr_lock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                if attr.value > 2.0:
                    self.modelock = True
                else:
                    self.modelock = False
                self.modelockLastUpdate = time.time()
            else:
                self.modelock = None
                self.set_state(PyTango.DevState.FAULT)
            self.waitingForModelock = False

    def updatePicomotor(self, arg):
        self.info_stream('Updating picomotor position')
        attr = arg.argout[0]
        with self.attr_lock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.picomotor_position = attr.value
                self.picomotorLastUpdate = time.time()
            else:
                self.picomotor_position = None
                self.set_state(PyTango.DevState.FAULT)
                self.set_status("Picomotor read INVALID")
            self.waitingForPicomotor = False

    def update_picomotor_write(self):
        with self.attr_lock:
            if self.waitingForPicomotorWrite is True:
                try:
                    self.picomotorDevice.command_inout_reply(self.waitingForPicomotorId)
                    self.waitingForPicomotorWrite = False
                    self.waitingForPicomotorId = None
                except PyTango.AsynReplyNotArrived:
                    pass

    def updateRedpitaya(self, arg):
        attr = arg.argout[0]
        with self.attr_lock:
            if attr.quality == PyTango.AttrQuality.ATTR_VALID:
                self.error_trace = attr.value
                # Jitter calculation according to kmlabs white paper
                fref = 2.9985e9
                Vpp = 0.8
                self.jitter = 1 / (2 * np.pi * fref) * self.error_trace.std() / (Vpp / 2)
                self.redpitayaLastUpdate = time.time()
            else:
                self.error_trace = None
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading error frequency')))
        with self.attr_lock:
            attr_read = self.error_frequency
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attr_lock:
            attr_read = self.picomotor_position
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_PicomotorPosition(self, attr):
        self.info_stream(''.join(('Writing picomotor position')))
        data = (attr.get_write_value())
        cmdMsg = HalcyonCommand('writeMotorPosition', data)
        self.command_queue.put(cmdMsg)

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
        with self.stream_lock:
            self.info_stream(''.join(('Reading picomotor follow')))
        with self.attr_lock:
            attr_read = self.halcyon_tracking.follow
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_PicomotorFollow(self, attr):
        self.info_stream(''.join(('Writing picomotor follow')))
        with self.attr_lock:
            data = (attr.get_write_value())
#            self.picomotorFollowParameters.follow = data
            self.info_stream(''.join(('Setting halcyonTracking.follow to ', str(data))))
            self.halcyon_tracking.follow = data

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
        with self.stream_lock:
            self.info_stream(''.join(('Reading picomotor position')))
        with self.attr_lock:
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading piezo voltage')))
        with self.attr_lock:
            attr_read = self.piezo_voltage
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading jitter')))
        with self.attr_lock:
            self.info_stream(''.join(('Jitter: ', str(self.jitter))))
            attr_read = self.jitter
            q = PyTango.AttrQuality.ATTR_VALID
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = 0.0
            if self.error_frequency > 0:
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading sample time')))
        with self.attr_lock:
            attr_read = self.sample_time
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
        with self.stream_lock:
            self.info_stream(''.join(('Reading ErrorTrace')))
        with self.attr_lock:
            attr_read = np.copy(self.error_trace)
            with self.stream_lock:
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read')))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read type ', str(type(attr_read)))))
                self.debug_stream(''.join(('In read_ErrorTrace: attr_read shape ', str(attr_read.shape))))
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = np.array([0.0])
                with self.stream_lock:
                    self.debug_stream(''.join(('In read_ErrorTrace: attr_read==None')))
            attr.set_value(attr_read, attr_read.shape[0])
        with self.stream_lock:
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
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmdMsg = HalcyonCommand('on')
        self.command_queue.put(cmdMsg)

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
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::Stop")))
        cmdMsg = HalcyonCommand('stop')
        self.command_queue.put(cmdMsg)

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
            [PyTango.DevString,
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
        'piezoLowVoltage':
            [PyTango.DevDouble,
             "Low setting of piezo voltage for auto adjusting with the picomotor, 0-100",
             [20]],
        'piezoHighVoltage':
            [PyTango.DevDouble,
             "High setting of piezo voltage for auto adjusting with the picomotor, 0-100",
             [80]],
        'piezoDeadBand':
            [PyTango.DevDouble,
             "Piezo voltage range in the center where no further adjustments are done",
             [10]],
        'maxNumberAdjusts':
            [PyTango.DevLong,
             "Maximum number of adjustment attempts done with picomotor before giving up",
             [30]],
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
