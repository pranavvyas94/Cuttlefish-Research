# -*- coding: utf-8 -*-

#Tuning the parameters:
    #Integrator max(resp .min): max nb of steps the motor can make in DeltaT

from collections import deque
import scipy.optimize as opt
import scipy.interpolate as interpolate
import scipy.signal as signal
import numpy as np

from control.optotune_lens import optotune_lens

import control.utils.image_processing as image_processing

from control._def import *


class Tracker_Focus():
    '''
    Liquid lens based tracker.
    '''
    def __init__(self,parent=None):
        
        self.YdequeLen=50
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)  #The length of the buffer must be ~fps_sampling/liquid_lens_freq
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        
        # kept for class compatibility
        self.YmaxFM = 0
        self.gain=1
        self.maxGain=1

        self.error = 0
        
        self.liquidLensAmp = liquidLens['Amp']['default']
        self.liquidLensFreq = liquidLens['Freq']['default']

        # Initialize a liquid lens
        self.liquid_lens = optotune_lens(freq = self.liquidLensFreq, amp = self.liquidLensAmp, offset = 0)


        self.window_size = 25
        
        # flags
        self.TrackingCycleStarted = 0
        self.TrackingCycleFinished = 0

        # cycle counter
        self.cycleCounter = 0

        # arrays used in the tracking cycle
        self.FocusMeasure = np.empty((0,0))
        self.Position = np.empty((0,0))
        
        # kept for class compatibility
        self.FM_slope = 0 # measured decay rate of FM with distance (Delta FM/ Delta mm)
        
        freq_range=[liquidLens['Freq']['min'],liquidLens['Freq']['max']]
        phase_lag = [0,0]
        self.phase_lag_funct =interpolate.interp1d(freq_range,phase_lag)
        self.phase_lag = self.phase_lag_funct(self.liquidLensFreq)

        self.cropped_imSize = None

    def save_list(self):
        # List of variables from this object that need to be saved
        save_list = ['YfocusPhase', 'liquidLensFreq', 'liquidLensAmp']

        return save_list

    
    def update_data(self,phase):
        self.YfocusPhase.append(phase)
        self.YfocusPosition.append(self.liquidLensAmp*np.sin(phase))

    def get_focus_error(self,image, centroid):
        self.isFocusOrder = 0
        Y_order=0
        # Crop the image:
        cropped_image = image_processing.crop(image, centroid, self.cropped_imSize)

        
        focusMeasure = image_processing.YTracking_Objective_Function(cropped_image, self.color)
        Y_order,self.isFocusOrder = self.tracker_y.get_error(focusMeasure)
        # Disabling Y tracking for 3D PIV
        # self.isYorder = 0
        return Y_order

    def get_error(self, focusMeasure):

        self.YfocusMeasure.append(focusMeasure)
        focusMeasure_list=np.array(self.YfocusMeasure)
        focusPosition_list=np.array(self.YfocusPosition)
        focusPhase_list=np.array(self.YfocusPhase)
        isYorder = 0

        if len(focusMeasure_list) < self.window_size:
            
            # fill the buffer first
            isYorder = 0

        else:
            
            # arduino firmware ensures that phase is between 0 and 2*pi
            # set check if a new sweep cycle has started
            if focusPhase_list[-1] < focusPhase_list[-2]:
                
                self.cycleCounter = self.cycleCounter % 2
                print('tracking cycle: ' + str(self.cycleCounter)) # @@@
                
                if self.cycleCounter == 0:
                    
                    self.TrackingCycleStarted = 1
                    self.FocusMeasure = np.empty((0,0))
                    self.Position = np.empty((0,0))
                
                elif self.cycleCounter == 1:

                    self.TrackingCycleStarted = 0
                    self.TrackingCycleFinished = 1
                
                # do nothing for the remaining cycles
                        
                self.cycleCounter = self.cycleCounter + 1
            
            # in the tracking cycle
            if self.TrackingCycleStarted:
            
                # record focus measure and position
                self.FocusMeasure = np.append(self.FocusMeasure,focusMeasure_list[-1])
                self.Position = np.append(self.Position,focusPosition_list[-1])
            
            # after the tracking cycle is finished
            if self.TrackingCycleFinished:
                
                self.TrackingCycleFinished = 0
                # now it's time to calculate focus error
                idx = np.argmax(self.FocusMeasure)
                print('Max FM: {}'.format(max(self.FocusMeasure)))
                self.error = self.Position[idx]
                isYorder = 1

        print('Y-error: {}'.format(self.error))
        return -self.error, isYorder

    #@@@
    def resize_buffers(self,buffer_size):
        self.YdequeLen=buffer_size
        self.YfocusMeasure=deque(self.YfocusMeasure,maxlen = self.YdequeLen)
        self.YfocusPosition=deque(self.YfocusPosition,maxlen = self.YdequeLen)
        self.YfocusPhase=deque(self.YfocusPhase,maxlen = self.YdequeLen)
        self.YmaxFM=deque(self.YmaxFM,maxlen = self.YdequeLen)
        
    def set_Amp(self,liquidLensAmp):
        self.liquidLensAmp=liquidLensAmp
        print('liquid Lens amp: {}'.format(self.liquidLensAmp))
            
    def set_Freq(self,liquidLensFreq):
        # print("Frequency: {}".format(liquidLensFreq))
        self.liquidLensFreq = liquidLensFreq
        self.phase_lag = self.phase_lag_funct(self.liquidLensFreq)
        print('liquid Lens freq: {}'.format(self.liquidLensFreq))
        
    def set_maxGain(self,gain):
        self.maxGain=gain



    def initialise_ytracking(self):
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        self.TrackingCycleStarted = 0 # @@@ rename
        self.TrackingCycleFinished = 0 # @@@ rename
        self.cycleCounter = 0





