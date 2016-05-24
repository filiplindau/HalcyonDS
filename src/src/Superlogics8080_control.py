'''
Created on 30 jan 2015

@author: Filip Lindau
'''

import serial

class Superlogics8080_control:
    def __init__(self, port):
        self.port = port
        try:
            self.close()
        except Exception:
            pass
        self.connected = False
        self.s = None
        self.connect()
        conf = self.getConfiguration()
        if conf == '!01510600':
            print 'Configuration ok'
        else:
            print ''.join(('Configuration problem: ', conf))
        
    def close(self):
        if self.s != None:
            try:
                self.s.close()
            except Exception:
                pass
        self.s = None
        self.connected = False
        
    def connect(self):
        self.close()
        self.s = serial.Serial(self.port, 9600, timeout=0.5, writeTimeout=0.5)
        self.connected = True
            
    def sendReceive(self, cmd):
        c = ''.join((cmd, '\r'))
        retries = 0
        writeReady = False
        while writeReady == False:
            try:
                self.s.write(c)
                writeReady = True
            except Exception:
                self.connect()
                retries += 1
                if retries > 4:
                    raise
        
        readBuf = ''
        readChr = ''
        maxLength = 15
        while readChr != '\r' and readBuf.__len__() < maxLength:
            readChr = self.s.read(1)
            if readChr == '':
                break
            if readChr != '\r':
                readBuf = ''.join((readBuf, readChr))
        if readBuf.__len__() >= maxLength:
            raise(serial.SerialException(''.join(('Received message too long: ', readBuf))))
        if readBuf.__len__() == 0:
            raise(serial.SerialException('Nothing received'))
        return readBuf
    
    def getConfiguration(self):
        cmd = '$012'
        conf = self.sendReceive(cmd)
        return conf
    
    def getInputMode(self):
        cmd = '$01B'
        resp = self.sendReceive(cmd)  
        inMode = int(resp[-1])
        if inMode == 0:
            s = 'ch1: non-isolated, ch2: non-isolated'
        elif inMode == 1:
            s = 'ch1: isolated, ch2: isolated'
        elif inMode == 2:
            s = 'ch1: non-isolated, ch2: isolated'
        elif inMode == 3:
            s = 'ch1: isolated, ch2: non-isolated'
        else:
            s = 'unknown mode'
        return (inMode, s)

    def getFrequency(self):
        cmd = '#010'
        resp = self.sendReceive(cmd)  
        freq = int(resp[1:], 16)      
        return freq
        
if __name__ == '__main__':
    sc = Superlogics8080_control('com13')