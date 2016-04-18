import time
import logging
import struct
import numpy as np

from i2c_per import I2C

logger = logging.getLogger()
f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
logger.addHandler(fh)
logger.setLevel(logging.DEBUG)


class AD7991Control(object):
    def __init__(self, address=0x28, bus=1):
        """
        Control of AD7991 thorugh i2c using the smbus package.

        :param address: i2c address of the device (default 0x28 for AD7991)
        :param bus: i2c bus connected (bus 1 for the raspberry)
        """
        self.bus_name = ''.join(('/dev/i2c-', str(bus)))
        self.addr = address

        self.ref_sel = 0        # 0 = vcc as reference, 1 = external reference on vin3
        self.bit_trial_delay = 0
        self.sample_delay = 0
        self.fltr = 0
        self.channel_enable = [1, 0, 0, 0]

    def compile_config(self):
        config = 0
        config += self.sample_delay
        config += self.bit_trial_delay << 1
        config += self.fltr << 2
        config += self.ref_sel << 3
        config += self.channel_enable[0] << 4
        config += self.channel_enable[1] << 5
        config += self.channel_enable[2] << 6
        config += self.channel_enable[3] << 7
        return config

    def write_config(self, config):
        try:
            data = [np.uint8(config)]
            msg = [I2C.Message(data, read=False, flags=0)]
            with I2C(self.bus_name) as i2c:
                i2c.transfer(self.addr, msg)
            logger.debug(''.join(('Writing config ', str(config))))
        except IOError, e:
            logger.error(''.join(('Error writing config, ', str(e))))

    def read_ad_result(self):
        try:
            msg = [I2C.Message(bytearray(2), read=True, flags=0)]
            with I2C(self.bus_name) as i2c:
                i2c.transfer(self.addr, msg)
            data = struct.unpack('>h', msg[0].data)[0]
            logger.debug(''.join(('Returned ', str(data), ' ', str(bin(data)))))
            ch = data >> 12     # Mask out channel bits
            ad = data & 0b0000111111111111  # Mask out first 12 bits
            return (ch, ad)
        except IOError, e:
            logger.error(''.join(('Error writing config, ', str(e))))

    def set_channel_enable(self, channel, enable):
        if 0 <= channel < 4:
            if enable == 1:
                self.channel_enable[channel] = 1
            else:
                self.channel_enable[channel] = 0
            self.write_config(self.compile_config())
        else:
            raise ValueError(''.join(('Channel must be 0..3')))

    def set_reference(self, ref):
        ''' Set which reference voltage to use.

        input:
            ref = 'internal' or 'vcc'... use vcc (3.3V) as reference
            ref = 'external'... use external voltage reference on pin Vin3
        '''
        s = str(ref).lower()
        if s in ['internal', 'vcc', 'int']:
            self.ref_sel = 0
        elif s in ['external', 'ext']:
            self.ref_sel = 1
        else:
            raise ValueError('Ref must be internal (or vcc), or external')
        self.write_config(self.compile_config())


if __name__ == '__main__':
    ad7991 = AD7991Control(0x28)
    ad7991.set_reference('external')
    logger.info(''.join(('External ref: ', str(ad7991.read_ad_result()))))
    ad7991.set_reference('internal')
    logger.info(''.join(('Internal ref: ', str(ad7991.read_ad_result()))))
