#!/usr/bin/env python
import sys
import time
from math import *
import serial
import traceback
import numpy as np 
import cv2
import datetime
from threading import Thread, Lock  
from mss import mss

class ImageCapturer():
    def __init__(self, config, ledProcessor): 
        self.cam = mss() 
        self.config = config  
        print self.cam.monitors
        self.displayResolution = {'mon':0,"top": 0, "left": 0, "width": self.config['screenWidth'], "height": self.config['screenHeight']} #self.cam.monitors[0] 
  
        self.image = None 
        self.LedProcessor = ledProcessor
        self.LedProcessor.start() 
    def run(self):
        try:
            while True: 
                #valid, img = self.cam.read()  
                img = np.array(self.cam.grab(self.displayResolution))
                
                if img is not None: 
                    self.image = cv2.resize(img, (self.config['ledsHorizontally'] ,self.config['ledsVertically']), cv2.INTER_NEAREST)  
                    self.LedProcessor.updateImage(self.image.copy())
                    time.sleep(self.config['mssSleepTime'])  
        except:
            traceback.print_exc() 
              
             
class AmbientLedProcessor(Thread):
    def __init__(self, config):
        Thread.__init__(self)
        self.daemon = True 
        self.SerialPort = None 
        self.image = None
        self.config = config
        self.Mem =  np.zeros((config['memCount'],config['LedsCount'],3), dtype=np.uint8)
        self.SerialHeader = None  
        self.initSerialConnection()
        self.SerialHeader = self.createHeader()    
        self.avgStd = 0
        self.setColor((255,255,255))
    def setColor(self, c):
            (r,g,b) = c
            leds = [r,g,b]
            leds_list = self.SerialHeader + leds * self.config['LedsCount']
            self.SerialPort.write(leds_list)  
    def updateImage(self,image):
        self.image = image
    def initSerialConnection(self):
        try:
            self.SerialPort = serial.Serial(
                self.config['comPort'], 
                self.config['baudrate'], 
                timeout = 5)
            ada = self.SerialPort.readline()
            print "Light Initialized" if "Ada" in ada else "connection shakehand error"
        except:
            traceback.print_exc()
            self.SerialPort.close()

    def createHeader(self):
        ledsMaxID = self.config['LedsCount'] - 1
        header = []
        header.append('A')
        header.append('d')
        header.append('a')
        # LED count high byte
        header.append(chr((ledsMaxID) >> 8))
        # LED count low byte
        header.append(chr((ledsMaxID) & 0xff))
        header.append(chr(ord(header[3]) ^ ord(header[4]) ^ 0x55))  # Checksum
        return header
 
    def getSectors(self): 
            imghsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV).astype("float32")
            (h, s, v) = cv2.split(imghsv)
               
            s = np.clip(s*5,0,255)       

            imghsv = cv2.merge([h,s,v]) 
            
            process_image =  cv2.cvtColor(imghsv.astype("uint8"), cv2.COLOR_HSV2RGB)
             
            rightVertical = process_image[:, -1]

            leftVertical = process_image[:, 0][::-1]

            topHorizontal = process_image[0, :]

            bottomHorizontal = process_image[-1, :][::-1]
 
            vertical = np.concatenate((leftVertical, topHorizontal))

            horizontal = np.concatenate((rightVertical, bottomHorizontal))

            sectors = np.concatenate((vertical, horizontal))
            return sectors

    def calculateLEDColors(self):
        processedRegions = []
        try:  
            if self.image is None :
                return 

            processedRegions = self.getSectors()

            self.Mem = np.vstack( ( [processedRegions], self.Mem[:-1]))  
  
            weightRange = range(len(self.Mem),0,-1) 
            avgStd = np.std( self.Mem )

            avgDif = avgStd  / self.avgStd if self.avgStd > 0 else 1

            if( avgDif > 1 ):
                weightRange[0] *= avgDif  
                weightRange =  np.clip(weightRange,0,255)
            
            self.avgStd  = avgStd
            
            processedRegions = np.round(np.average(self.Mem,axis=0,weights=weightRange ),0)#  np.mean( self.Errors)
                  

        except:
            traceback.print_exc()   
        return list(np.concatenate(processedRegions.astype(int)))

    def run(self):
        try:
            while True:#"Ada" in self.SerialPort.readline():

                leds = self.calculateLEDColors()
                if(leds is None):
                    print 'no image'
                    time.sleep(1)
                    continue
                leds_list = self.SerialHeader+leds
                self.SerialPort.write(leds_list) 
                serial.Serial.flush(self.SerialPort) 
        except: 
            self.SerialPort.close()
            traceback.print_exc()
  
if __name__ == '__main__': 
    config = {'LedsCount': 122, 'baudrate':115200,'comPort':'COM4','screenWidth':3840,'screenHeight':2160,'ledsHorizontally':39,'ledsVertically':22,'mssSleepTime':0.75, 'memCount':14} 
    _ledProcessor = AmbientLedProcessor(config) 
    capturer = ImageCapturer(config, _ledProcessor) 
    capturer.run()
    #thread.start_new_thread(AmbiLedInstance.worker, ())
