import argparse
import cv2
import time
import numpy as np
import os
try:
    import control.gxipy as gx
except:
    print('gxipy import error')

class Camera(object):

    def __init__(self,sn=None):

        # many to be purged
        self.sn = sn
        self.device_manager = gx.DeviceManager()
        self.device_info_list = None
        self.device_index = 0
        self.camera = None
        self.is_color = None
        self.gamma_lut = None
        self.contrast_lut = None
        self.color_correction_param = None

        self.exposure_time = 0
        self.analog_gain = 0
        self.frame_ID = -1
        self.timestamp = 0

        self.image_locked = False
        self.current_frame = None

        self.callback_is_enabled = False
        self.callback_was_enabled_before_autofocus = False
        self.callback_was_enabled_before_multipoint = False

        self.GAIN_MAX = 24
        self.GAIN_MIN = 0
        self.GAIN_STEP = 1
        self.EXPOSURE_TIME_MS_MIN = 0.01
        self.EXPOSURE_TIME_MS_MAX = 4000

    def open(self,index=0):
        (device_num, self.device_info_list) = self.device_manager.update_device_list()
        if device_num == 0:
            raise RuntimeError('Could not find any USB camera devices!')
        if self.sn is None:
            self.device_index = index
            self.camera = self.device_manager.open_device_by_index(index + 1)
        else:
            self.camera = self.device_manager.open_device_by_sn(self.sn)
        self.is_color = self.camera.PixelColorFilter.is_implemented()
        # self._update_image_improvement_params()
        # self.camera.register_capture_callback(self,self._on_frame_callback)
        if self.is_color:
            # self.set_wb_ratios(self.get_awb_ratios())
            print(self.get_awb_ratios())
            # self.set_wb_ratios(1.28125,1.0,2.9453125)
            self.set_wb_ratios(2,1,2)

    def set_callback(self,function):
        self.new_image_callback_external = function

    def enable_callback(self):
        user_param = None
        self.camera.register_capture_callback(user_param,self._on_frame_callback)
        self.callback_is_enabled = True

    def disable_callback(self):
        self.camera.unregister_capture_callback()
        self.callback_is_enabled = False

    def open_by_sn(self,sn):
        (device_num, self.device_info_list) = self.device_manager.update_device_list()
        if device_num == 0:
            raise RuntimeError('Could not find any USB camera devices!')
        self.camera = self.device_manager.open_device_by_sn(sn)
        self.is_color = self.camera.PixelColorFilter.is_implemented()
        self._update_image_improvement_params()

        '''
        if self.is_color is True:
            self.camera.register_capture_callback(_on_color_frame_callback)
        else:
            self.camera.register_capture_callback(_on_frame_callback)
        '''

    def close(self):
        self.camera.close_device()
        self.device_info_list = None
        self.camera = None
        self.is_color = None
        self.gamma_lut = None
        self.contrast_lut = None
        self.color_correction_param = None
        self.last_raw_image = None
        self.last_converted_image = None
        self.last_numpy_image = None

    def set_exposure_time(self,exposure_time):
        self.exposure_time = exposure_time
        self.camera.ExposureTime.set(exposure_time * 1000)

    def set_analog_gain(self,analog_gain):
        self.analog_gain = analog_gain
        self.camera.Gain.set(analog_gain)

    def get_awb_ratios(self):
        self.camera.BalanceWhiteAuto.set(2)
        self.camera.BalanceRatioSelector.set(0)
        awb_r = self.camera.BalanceRatio.get()
        self.camera.BalanceRatioSelector.set(1)
        awb_g = self.camera.BalanceRatio.get()
        self.camera.BalanceRatioSelector.set(2)
        awb_b = self.camera.BalanceRatio.get()
        return (awb_r, awb_g, awb_b)

    def set_wb_ratios(self, wb_r=None, wb_g=None, wb_b=None):
        self.camera.BalanceWhiteAuto.set(0)
        if wb_r is not None:
            self.camera.BalanceRatioSelector.set(0)
            awb_r = self.camera.BalanceRatio.set(wb_r)
        if wb_g is not None:
            self.camera.BalanceRatioSelector.set(1)
            awb_g = self.camera.BalanceRatio.set(wb_g)
        if wb_b is not None:
            self.camera.BalanceRatioSelector.set(2)
            awb_b = self.camera.BalanceRatio.set(wb_b)

    def start_streaming(self):
        self.camera.stream_on()

    def stop_streaming(self):
        self.camera.stream_off()

    def set_continuous_acquisition(self):
        self.camera.TriggerMode.set(gx.GxSwitchEntry.OFF)

    def set_software_triggered_acquisition(self):
        self.camera.TriggerMode.set(gx.GxSwitchEntry.ON)
        self.camera.TriggerSource.set(gx.GxTriggerSourceEntry.SOFTWARE)
        print('Set software triggered aquisition')

    def set_hardware_triggered_acquisition(self):
        self.camera.TriggerMode.set(gx.GxSwitchEntry.ON)
        self.camera.TriggerSource.set(gx.GxTriggerSourceEntry.LINE0)

    def send_trigger(self):
        # print("sending trigger to camera")
        self.camera.TriggerSoftware.send_command()

    def read_frame(self):
        raw_image = self.camera.data_stream[self.device_index].get_image()
        if self.is_color:
            rgb_image = raw_image.convert("RGB")
            numpy_image = rgb_image.get_numpy_array()
        else:
            numpy_image = raw_image.get_numpy_array()
        # self.current_frame = numpy_image
        return numpy_image

    def _on_frame_callback(self, user_param, raw_image):
        # print("In camera call back")
        if raw_image is None:
            print("Getting image failed.")
            return
        if raw_image.get_status() != 0:
            print("Got an incomplete frame")
            return
        if self.image_locked:
            print('last image is still being processed, a frame is dropped')
            return
        if self.is_color:
            rgb_image = raw_image.convert("RGB")
            numpy_image = rgb_image.get_numpy_array()
        else:
            numpy_image = raw_image.get_numpy_array()
        if numpy_image is None:
            return
        self.current_frame = numpy_image
        self.frame_ID = self.frame_ID + 1 # @@@ read frame ID from the camera
        self.timestamp = time.time()
        self.new_image_callback_external(self)
       
        # print(self.frame_ID)


class Camera_Simulation(object):

    def __init__(self,sn=None,width=2000,height=2000,framerate=30,color=False):
        self.height = height
        self.width = width
        self.sample = None
        self.samplelocked = False
        self.newsample = False
        self.gotimage = False
        self.img_mat = None
        self.new_image_callback_external = None
        self.image_locked = False
        self.is_streaming = False
        self.is_color = color

        self.GAIN_MAX = 480
        self.GAIN_MIN = 0
        self.GAIN_STEP = 1
        self.EXPOSURE_TIME_MS_MIN = 0.02
        self.EXPOSURE_TIME_MS_MAX = 4000

        # Path for getting an image stream from disk
        self.path = '/Users/deepak/Dropbox/GravityMachine/ExperimentResults/TestData/seacucmber4_auto_verylong_goodtrack/images'
        
        # self.path = '/Users/deepak/Dropbox/GravityMachine/ExperimentResults/TestData/Stentor'
        if(os.path.exists(self.path)):
            self.FileList = os.listdir(self.path)
    

    def open(self,index=0):
        pass

    def set_callback(self,function):
        self.new_image_callback_external = function

    def enable_callback(self):
        pass

    def disable_callback(self):
        pass

    def open_by_sn(self,sn):
        pass

    def close(self):
        pass

    def set_exposure_time(self,exposure_time):
        print('Set exposure time to: {}'.format(exposure_time))

    def set_analog_gain(self,analog_gain):
        print('Set analog gain to: {}'.format(analog_gain))

    def get_awb_ratios(self):
        pass

    def set_wb_ratios(self, wb_r=None, wb_g=None, wb_b=None):
        pass

    def start_streaming(self):
        self.frame_ID = 0

    def stop_streaming(self):
        pass

    def set_continuous_acquisition(self):
        pass

    def set_software_triggered_acquisition(self):
        pass

    def set_hardware_triggered_acquisition(self):
        pass

    def send_trigger(self):


        self.frame_ID = self.frame_ID + 1
        self.timestamp = time.time()
        if self.frame_ID == 1:
            if(self.is_color == False):
                self.current_frame = np.random.randint(50,size=(2000,2000),dtype=np.uint8)
                self.current_frame[800:1000,900:1100] = 200
                # self.current_frame[250:400,400:600] = 200
                cv2.circle(self.current_frame,(400,400), 100, (200,0,0), -1)

            elif(self.is_color == True):
                self.current_frame = np.random.randint(50,size=(2000,2000,3),dtype=np.uint8)
                self.current_frame[800:1000,900:1100,1] = 200
                self.current_frame[250:400,400:600,1] = 200
        else:
            self.current_frame = np.roll(self.current_frame,10,axis=1)
            # pass 
            # self.current_frame = np.random.randint(255,size=(768,1024),dtype=np.uint8)
        if self.new_image_callback_external is not None:
            self.new_image_callback_external(self)

    # def send_trigger(self):

    #     self.frame_ID = self.frame_ID + 1
    #     self.timestamp = time.time()

    #     if(self.frame_ID == len(self.FileList)):
    #         self.frame_ID = 0

    #     file = self.FileList[self.frame_ID]

    #     image = cv2.imread(os.path.join(self.path, file),0)
    #     self.current_frame = image

    #     if self.new_image_callback_external is not None:
    #         self.new_image_callback_external(self)

    def read_frame(self):
        pass

    def _on_new_buffer(self, appsink):
        pass

    def _get_property(self, PropertyName):
        pass

    def _set_property(self, PropertyName, value):
        pass

    def _gstbuffer_to_opencv(self):
        pass