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

import imutils

from sklearn.cluster import KMeans

from PIL import Image
 
from pyimagesearch.transform import four_point_transform

from skimage.filters import threshold_local

import os

import ast
 
class Settings:
     def __init__(self,configInitFile):
        self.mainPath  =  os.path.dirname(sys.argv[0]) + '\\'
        self.presetsPath  =  self.mainPath + '\\'  
        self.configInitFile = "init.ini" if configInitFile is None else configInitFile
        self.actualConfigFile = self.mainPath + self.configInitFile
        self.config = {}
     def LoadSettings(self): 
             
            if(os.path.isfile( self.actualConfigFile )):
                try:
                    f = open(  self.actualConfigFile , 'r')
                    data  = f.read()
               
                    _options = data.split("\n")
                    
                    for param in _options:
                         
                        if len( param ) == 0 : return -1
                        
                        p = param.split("=") 
                        desc = p[0].split(",")
                        name = desc[0]
                        typ = desc[1]
                        value = p[1] 
                        
                        if    typ == "f":
                                value = float(value)
                        elif  typ == "i": 
                                value = int(value)
                        elif typ  == "l" or typ == "t":
                                value = ast.literal_eval(value)
                         
                        self.config [name] = value
                        
                    print "config file has been loaded." 
                except IOError:
                    print 'cannot open config file.', self.actualConfigFile
                finally:
                    f.close()
                

     def SaveSettings(self):
  
            f = open( self.actualConfigFile, 'w')
            try:
                for key, value in self.config.iteritems(): 
                            
                    if  isinstance(value,float):
                            typ = "f"
                    elif  isinstance(value,int):
                            typ = "i"
                    elif type(value) is list:
                            typ = "l"
                    elif type(value) is tuple:
                            typ = "t"
                    else:   typ = "s"
                    
                    param = '{0},{1}={2}\n'.format(key,typ, value)
                                
                    f.write(param)
            except:
                print 'error'
            finally:
                f.close() 


class ImageCapturer():

    def __init__(self, settings, ledProcessor):  
        self.settings = settings
        self.config = settings.config
        self.cam = cv2.VideoCapture(0)  
        self.init_image("image") 
        #self.cam.set(13, self.config['hue'])    
        self.cam.set(14, self.config['gain'])
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0 )

        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 1)  
        self.cam.set(3, self.config['width'])

        self.cam.set(4, self.config['height'])

        self.cam.set(5 , 60) 

        self.dimensions = (self.config['width'], self.config['height']) 

        self.width, self.height = self.dimensions 

        self.image = None
        self.previewMultiplier = 2
        self.imageReshaped = None

        self.LedProcessor = ledProcessor

        self.showPreview = True

        self.LedProcessor.start()

    def init_image(self,name):
        cv2.destroyAllWindows()
        cv2.namedWindow(name)
        cv2.setMouseCallback(name, self.click_and_crop)

    def click_and_crop(self,event, x, y, flags, param): 
        if (len(self.config['refPt'])==5):
            cv2.destroyAllWindows()
            self.init_image("image")
            self.config['refPt'] = [] 
 
        if event == cv2.EVENT_RBUTTONUP:
             self.showPreview = not self.showPreview
        if event == cv2.EVENT_LBUTTONUP: 
            self.config['refPt'].append([x/self.previewMultiplier , y / self.previewMultiplier])
            if(len(self.config['refPt']) == 4): 
                self.settings.SaveSettings()
                self.init_image("image") 
 
    def run(self):

        try:

            while True: 

                valid,img = self.cam.read()  

                if valid:  
                    self.image =  img
 
                    if len(self.config['refPt'])==4: 
                        
                        pts = np.array(self.config['refPt'], np.int32)
                        pts = pts.reshape((-1,1,2))
                        self.imageReshaped = four_point_transform(self.image, pts.reshape(4, 2)  )
                        cv2.polylines(self.image, [pts], True,(255,255,255),thickness = 2)
                        if self.showPreview: 
                             
                            original_image_size  = (self.image.shape[1],self.image.shape[0])
                            normalized_reshape = cv2.resize(self.imageReshaped, original_image_size, cv2.INTER_LANCZOS4)
                            numpy_horizontal_concat = np.concatenate((self.image, normalized_reshape ), axis=1)
                            
                            cv2.imshow("image",  numpy_horizontal_concat )
                    else:
                        if self.showPreview: 
                            cv2.imshow("image", cv2.resize(self.image, (self.previewMultiplier*self.image.shape[1],self.previewMultiplier*self.image.shape[0]) )  )

                    if cv2.waitKey(1) & 0xFF == ord('q'): 
                            break
                    if self.imageReshaped is not None:
                        self.image =  cv2.resize(self.imageReshaped, self.dimensions, cv2.INTER_LANCZOS4)    
                        self.LedProcessor.updateImage( self.image )
 
        except:

            traceback.print_exc() 

            self.cam.release()  

             

class AmbientLedProcessor(Thread):

    def __init__(self, config):

        Thread.__init__(self)

        self.daemon = True 
        
        self.working = False

        self.SerialPort = None 

        self.image = None

        self.settings = settings

        self.config = settings.config

        self.Mem =  np.zeros((self.config['memCount'],self.config['LedsCount'],3), dtype=np.uint8)

        self.SerialHeader = None 

        self.dimensions = (self.config['width'],self.config['height']) 

        width, height = self.dimensions

        self.sectorWidth = int( round((2.0*(width+height))/self.config['LedsCount']) )


        self.initSerialConnection()

        self.SerialHeader = self.createHeader()   

        self.setColor((0,0,0))
        self.avgStd = 0
        self.longCount = 22#int(round(height/self.sectorWidth))

        self.wideCount = 39#int(round(width/self.sectorWidth))

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

 

    def getSectors(self, data):

            img,longCount,wideCount = data

            #clearImg = cv2.fastNlMeansDenoisingColored(img,None,10,10,7,21) -- TOOO SLOW

            minimized_image = cv2.resize(img, (11,16 ), cv2.INTER_LANCZOS4)
            resized_image = cv2.resize(minimized_image, (longCount,wideCount ), cv2.INTER_NEAREST)
            #resized_image = self.convert_temp(resized_image,8000)
            process_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB) 
            '''
            _img = resized_image.copy().reshape((resized_image.shape[0] * resized_image.shape[1], 3))
            self.CLUSTERS = 5
            kmeans = KMeans(n_clusters = self.CLUSTERS)
            kmeans.fit(_img)
            self.COLORS = kmeans.cluster_centers_
            self.LABELS = kmeans.labels_  
            numLabels = np.arange(0, self.CLUSTERS+1)
        
            #create frequency count tables    
            (hist, _) = np.histogram(self.LABELS, bins = numLabels)
            hist = hist.astype("float")
            hist /= hist.sum()
            colors = self.COLORS[(-hist).argsort()].astype('uint8')
            color_std = np.std(colors)
            
            hist = hist[(-hist).argsort()] 
            b,g,r  = colors[0]              
            
             
            #imrgb = np.array(self.convert_temp(resized_image,7000)) 
            #cv2.imshow('resized',imrgb)

            #
            '''
            
            imghsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV).astype("float32")
            (h, s, v) = cv2.split(imghsv)
             
            h = np.clip(np.add(h, -5),0,255)       
            s = np.clip(s*1.25,0,255)       
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

            img = self.image
 

            if img is None :

                return
 
            processedRegions = self.getSectors((img, self.wideCount, self.longCount))
 
            self.Mem = np.vstack( ( [processedRegions], self.Mem[:-1])) 
  
            weightRange = range(len(self.Mem),0,-1)
            ''''''
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


    settings = Settings('init.ini') 
    settings.LoadSettings() 
    _ledProcessor = AmbientLedProcessor(settings) 

    capturer = ImageCapturer(settings, _ledProcessor) 

    capturer.run()

    #thread.start_new_thread(AmbiLedInstance.worker, ())

