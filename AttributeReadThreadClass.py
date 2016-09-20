"""
Created on 14 aug 2014

@author: Filip
"""
# -*- coding:utf-8 -*-


import time
import sys
import threading
import PyTango as pt


class DeviceWatchdogClass(object):
    def __init__(self, device_name, device=None, timeout=10.0):
        self.timeout = timeout
        self.device_name = device_name
        self.device = device

        if self.device is None:
            self.init_device()

        self.watchdog = None

        self.lock = threading.Lock()

    def init_device(self):
        with self.lock:
            self.device = pt.DeviceProxy(self.device_name)

    def reset_watchdog(self):
        with self.lock:
            if self.watchdog is not None:
                self.watchdog.cancel()
            self.watchdog = threading.Timer(self.timeout, self.timeout_watchdog)

    def timeout_watchdog(self):
        self.init_device()
        self.reset_watchdog()

    def stop_watchdog(self):
        with self.lock:
            if self.watchdog is not None:
                self.watchdog.cancel()


class AttributeClass(object):

    def __init__(self, name, device, interval, getInfo=False):
        super(AttributeClass, self).__init__()
        self.name = name
        self.device = device
        self.interval = interval
        self.get_info_flag = getInfo
        self._wvalue = None
        self._device_command = None
        
        self.read_event = threading.Event()
        self.attr_lock = threading.Lock()

        self.last_read = time.time()
        self.attr = None
        self.read_thread = threading.Thread(name=self.name, target=self.attr_read)
        self.stop_thread = False

        self.start_read()

    def attr_read(self):
        reply_ready = True
        while self.stop_thread is False:
            if self.get_info_flag is True:
                self.get_info_flag = False
                try:
                    self.attr_info = self.device.get_attribute_config(self.name)

                except pt.DevFailed, e:
                    if e[0].reason == 'API_DeviceTimeOut':
                        print 'AttrInfo Timeout'
                    else:
                        print self.name, '  attrinfo error ', e[0].reason
                    self.attr_info = pt.AttributeInfoEx()
                except Exception, e:
                    print self.name, ' recovering from attrInfo ', str(e)
                    self.attr_info = pt.AttributeInfoEx()

            t = time.time()

            if t-self.last_read > self.interval:
                self.last_read = t
                try:
                    read_attr_id = self.device.read_attribute_asynch(self.name)

                    reply_ready = False
                except pt.DevFailed, e:
                    if e[0].reason == 'API_DeviceTimeOut':
                        print 'Timeout'
                    else:
                        print self.name, ' error ', e[0].reason
                    with self.attr_lock:
                        self.attr = pt.DeviceAttribute()
                        self.attr.quality = pt.AttrQuality.ATTR_INVALID
                        self.attr.value = None
                        self.attr.w_value = None
                        self.read_event.set()

                except Exception, e:
                    print self.name, ' recovering from ', str(e)
                    with self.attr_lock:
                        self.attr = pt.DeviceAttribute()
                        self.attr.quality = pt.AttrQuality.ATTR_INVALID
                        self.attr.value = None
                        self.read_event.set()

                while reply_ready is False and self.stop_thread is False:
                    try:
                        with self.attr_lock:
                            self.attr = self.device.read_attribute_reply(read_attr_id)
                        reply_ready = True
                        self.read_event.set()
                        # print 'signal emitted', self.attr.value.shape
                        # Read only once if interval = None:
                        if self.interval is None:
                            self.stop_thread = True
                            self.interval = 0.0
                    except Exception, e:
                        if e[0].reason == 'API_AsynReplyNotArrived':
                            # print self.name, ' not replied'
                            time.sleep(0.1)
                        else:
                            reply_ready = True
                            print 'Error reply ', self.name, str(e)
                            with self.attr_lock:
                                self.attr = pt.DeviceAttribute()
                                self.attr.quality = pt.AttrQuality.ATTR_INVALID
                                self.attr.value = None
                                self.attr.w_value = None
                                self.read_event.set()

            if self._device_command is not None:
                with self.attr_lock:
                    cmd = self._device_command[0]
                    cmd_arg = self._device_command[1]
                    self._device_command = None
                    try:
                        if cmd_arg is None:
                            self.device.command_inout(cmd)
                        else:
                            self.device.command_inout(cmd, cmd_arg)
                    except:
                        pass

            if self._wvalue is not None:
                with self.attr_lock:
                    try:
                        self.device.write_attribute(self.name, self._wvalue)
                    except:
                        pass
                    finally:
                        self._wvalue = None

            if self.interval is not None:
                time.sleep(self.interval)
            else:
                time.sleep(1)
        print self.name, ' waiting for final reply'
        final_timeout = 1.0  # Wait max 1 s
        final_start_time = time.time()
        final_timeout_flag = False
        while reply_ready is False and final_timeout_flag is False:
            try:
                with self.attr_lock:
                    self.attr = self.device.read_attribute_reply(read_attr_id)
                reply_ready = True
                self.read_event.set()
            except Exception, e:
                if e[0].reason == 'API_AsynReplyNotArrived':
                    # print self.name, ' not replied'
                    time.sleep(0.1)
                else:
                    reply_ready = True
                    print 'Error reply ', self.name, str(e)
                    with self.attr_lock:
                        self.attr = pt.DeviceAttribute()
                        self.attr.quality = pt.AttrQuality.ATTR_INVALID
                        self.attr.value = None
                        self.attr.w_value = None
                        self.read_event.set()
            if time.time()-final_start_time > final_timeout:
                final_timeout_flag = True
        if final_timeout_flag is False:
            print self.name, '... Thread stopped'
        else:
            print self.name, '... Thread timed out'

    def attr_write(self, wvalue):
        with self.attr_lock:
            self._wvalue = wvalue

    def device_command(self, cmd, cmd_arg=None):
        """ Issue a command on the underlying device.
        This is a locking command, so the thread will be unresponsive for the duration"""
        with self.attr_lock:
            self._device_command = (cmd, cmd_arg)

    def stop_read(self, wait=True):
        self.stop_thread = True
        if wait is True:
            if self.read_thread.is_alive() is True:
                self.read_thread.join(3)

    def start_read(self):
        self.stop_read()
        self.stop_thread = False
        self.read_thread.start()

    def get_info(self):
        self.get_info_flag = True

    def get_attribute(self):
        self.read_event.clear()
        with self.attr_lock:
            return self.attr
