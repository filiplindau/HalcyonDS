"""
Created on 07 apr 2016

Contains classes for controlling the picomotor tracking of the Halcyon lock.

@author: Filip Lindau
"""
import time
import logging
import numpy as np

logging.basicConfig(format='%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s', level=logging.INFO)


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


class HalcyonCommand:
    def __init__(self, command, data=None):
        self.command = command
        self.data = data


class HalcyonDriver:
    def __init__(self):
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.moving = [False, False, False]


class HalcyonTracking(object):
    def __init__(self, cmdQueue):
        # Configuration parameters:
        self.follow = False
        self.search = False
        self.maxPiezoVoltage = 70
        self.minPiezoVoltage = 30
        self.centerPiezoVoltage = 50
        self.piezoDeadband = 10
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
        """ Function for normal state when the laser is modelocked and frequency locked,
        and also within piezo limits. Checks status of modelock, freqlock, and piezo."""
        logging.debug('Entering idleHandler')
        if self.modelock is False or self.modelock is None:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency > 0 or self.errorFrequency is None:
            self.halcyonState = 'nofreqlock'
        elif self.piezoVoltage < self.minPiezoVoltage or self.piezoVoltage > self.maxPiezoVoltage:
            logging.info('Idle: Piezo in adjust range')
            if self.follow is True:
                if self.lockdown is True:
                    # If the device was in lockdown mode
                    # and then was turned back to follow (someone issued
                    # wrote follow=True) we exit lockdown mode.
                    logging.info("Exit lockdown")
                    self.lockdown = False
                    cmdMsg = HalcyonCommand('on')
                    self.q.put(cmdMsg)
                logging.info('Following')
                self.adjustNumber = 0
                self.halcyonState = 'adjusting'
            else:
                logging.info('Not following')
                cmdMsg = HalcyonCommand('alarm', 'Locked, not tracking picomotor.')
                self.q.put(cmdMsg)
        else:
            cmdMsg = HalcyonCommand('on', 'Locked, picomotor idle')
            self.q.put(cmdMsg)

    def noModelockHandler(self):
        logging.info('Entering noModeLockHandler')
        cmdMsg = HalcyonCommand('alarm', 'No modelock')
        self.q.put(cmdMsg)
        if self.modelock is True:
            self.halcyonState = 'nofreqlock'

    def noFreqLockHandler(self):
        logging.info('Entering noFreqLockHandler')
        cmdMsg = HalcyonCommand('alarm', 'No frequency lock')
        self.q.put(cmdMsg)
        if self.modelock is False or self.modelock is None:
            # If we dropped out of modelock or the modelock device is not responding
            # go back to state 'nomodelock'
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency == 0:
            self.halcyonState = 'idle'
        elif self.search is True:
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
        cmdMsg = HalcyonCommand('alarm', 'Adjustment lockdown. Too many attempts.')
        self.q.put(cmdMsg)
        if self.modelock is False:
            self.halcyonState = 'nomodelock'

    def adjustingHandler(self):
        logging.info('Entering adjustingHandler')
        logging.info(''.join(('Adjustment #', str(self.adjustNumber))))
        if self.modelock is False or self.modelock is None:
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency > 0 or self.errorFrequency is None:
            self.halcyonState = 'nofreqlock'
        elif self.follow is True:
            if self.adjustNumber < self.maxNumberAdjusts:
                logging.info('Adjustment number ok')
                t = time.time()
                if t - self.lastUpdateTime > self.updateDeadtime:
                    logging.info('Adjustment time ok')

                    self.adjustNumber += 1
                    self.lastUpdateTime = t
                    # Only keep adjusting if we are outside a +- 5 range of center:
                    if np.abs(self.piezoVoltage - self.centerPiezoVoltage) > self.piezoDeadband/2:
                        logging.info('Adjustment voltage in range')
                        # Post a command that we are adjusting
                        cmdMsg = HalcyonCommand('adjust', 'Adjusting picomotor')
                        self.q.put(cmdMsg)

                        if np.abs(self.piezoVoltage) > 1:
                            # If the piezo voltage is too low there is a problem,
                            # probably a missing signal
                            if self.piezoVoltage < self.centerPiezoVoltage:
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
                    cmdMsg = HalcyonCommand('alarm', 'Entering lockdown')
                    self.q.put(cmdMsg)

    def searchingHandler(self):
        """ Function for searching out a frequency lock by moving the picomotor
        Currently does nothing."""
        logging.info('Entering searchingHandler')
        cmdMsg = HalcyonCommand('moving', 'Searching for frequency lock.')
        self.q.put(cmdMsg)
        if self.modelock is False or self.modelock is None:
            # If we dropped out of modelock or the modelock device is not responding
            # go back to state 'nomodelock'
            self.halcyonState = 'nomodelock'
        elif self.errorFrequency == 0:
            # If we have frequency lock go to state 'idle'
            self.halcyonState = 'idle'
