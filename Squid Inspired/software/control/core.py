'''
core objects that run the back-end of the GUI.
Key objects:

    1. StreamHandler

'''

# set QT_API environment variable
import os, sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *
import control.tracking as tracking

import control.utils.image_processing as image_processing


from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2
import imutils
from datetime import datetime

class StreamHandler(QObject):
    ''' Signals 
    '''
    image_to_display = Signal(np.ndarray, str)
    thresh_image_to_display = Signal(np.ndarray)
    packet_image_to_write = Signal(np.ndarray, int, float)
    packet_image_for_tracking = Signal(np.ndarray, np.ndarray)
    signal_new_frame_received = Signal()
    signal_fps = Signal(int)
    signal_fps_display = Signal(float)
    signal_fps_save = Signal(str, float)
    signal_working_resolution = Signal(int)

    '''
    Signals
    image_to_display ->ImageDisplayer.enque
    packet_image_to_write ->ImageSaver
    packet_image_for_tracking -> Tracking_controller.on_new_frame
    signal_new_frame_received -> microcontroller_Receiver.get_Data

    Slots
    '''

    def __init__(self, camera = None , crop_width=2000,crop_height=2000, working_resolution_scaling = WORKING_RES_DEFAULT, imaging_channel = TRACKING):
        QObject.__init__(self)
        self.fps_display = 1
        self.fps_save = 1
        self.fps_track = 1
        self.timestamp_last_display = 0
        self.timestamp_last_save = 0
        self.timestamp_last_track = 0

        self.crop_width = crop_width
        self.crop_height = crop_height
        self.working_resolution_scaling = working_resolution_scaling

        self.camera = camera

        self.save_image_flag = False

        # If current image stream is used for tracking.
        self.imaging_channel = imaging_channel

        self.track_flag = False

        self.invert_image_flag = False

        self.handler_busy = False

        # for fps measurement
        self.timestamp_last = 0
        self.counter = 0
        self.fps_real = 0

        self.fps_display_real = 0
        self.counter_display = 0
        self.timestamp_last_display_real = 0

        self.fps_save_real = 0
        self.counter_save = 0

        # Image thresholding parameters
        self.lower_HSV = np.array([0, 0, 100],dtype='uint8') 
        self.upper_HSV = np.array([255, 255, 255],dtype='uint8') 

       

    def start_recording(self):
        self.save_image_flag = True
        print('Starting Acquisition')

    def stop_recording(self):
        self.save_image_flag = False
        print('Stopping Acquisition')

    def start_tracking(self):
        self.track_flag = True

    def stop_tracking(self):
        self.track_flag = False

    def set_display_fps(self,fps):
        self.fps_display = fps
        #@@@Testing
        print(self.fps_display)

    def set_save_fps(self,fps):
        self.fps_save = fps
        print(self.fps_save)

    def set_crop(self,crop_width,height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_working_resolution_scaling(self, working_resolution_scaling):
        self.working_resolution_scaling = working_resolution_scaling/100
        # print(self.working_resolution_scaling)

    def set_image_thresholds(self, lower_HSV, upper_HSV):
        self.lower_HSV = lower_HSV
        self.upper_HSV = upper_HSV

        #@@@Testing
        print('Updated color thresholds to {} and {}'.format(self.lower_HSV, self.upper_HSV))

    def update_invert_image_flag(self, flag):
        self.invert_image_flag = flag
        
    def threshold_image(self, image_resized, color):
        if(color):
            thresh_image = image_processing.threshold_image(image_resized,self.lower_HSV,self.upper_HSV)  #The threshold image as one channel

        else:
            # print(self.lower_HSV[2])
            # print(self.upper_HSV[2])
            # img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # print(max(image_resized))
            # print(min(image_resized))
            image_resized = np.array(image_resized, dtype='uint8')
            thresh_image = image_processing.threshold_image_gray(image_resized, self.lower_HSV[2], self.upper_HSV[2])
            if(self.invert_image_flag==True):
                thresh_image = 1 - thresh_image

        return thresh_image

    def get_real_stream_fps(self):
        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            # print('real camera fps is ' + str(self.fps_real))

            self.signal_fps.emit(self.fps_real)

    def get_real_display_fps(self):
        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last_display_real:
            self.counter_display = self.counter_display+1
        else:
            self.timestamp_last_display_real = timestamp_now
            self.fps_display_real = self.counter_display
            self.counter_display = 0
            # print('real display fps is ' + str(self.fps_display_real))

            self.signal_fps_display.emit(self.fps_display_real)


    def on_new_frame(self, camera):

        # print('On new frame')
        camera.image_locked = True
        self.handler_busy = True
        self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()
        # This also triggers the microcontroller_Receiever

        
        self.get_real_stream_fps()

        # crop image
        image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
        # image = camera.current_frame

        # save a copy of full-res image for saving (make sure to do a deep copy)
        # @@@@@@@@@

        
        if(self.imaging_channel == TRACKING):
            image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
            # image_resized = imutils.resize(image, self.working_image_width)

            # Threshold the image based on the color-thresholds
            image_thresh = 255*np.array(self.threshold_image(image_resized, color = camera.is_color), dtype = 'uint8')

        else:

            image_resized = np.copy(image)
        
        # Deepak: For now tracking with every image from camera
        time_now = time.time() 
        if self.track_flag and self.imaging_channel == TRACKING:
            # track is a blocking operation - it needs to be
            # @@@ will cropping before emitting the signal lead to speedup?

            self.packet_image_for_tracking.emit(image_resized, image_thresh)
            self.timestamp_last_track = time_now

        # send image to display
        time_now = time.time()
        if time_now - self.timestamp_last_display >= 1/self.fps_display:

            if camera.is_color:
                image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

            # print('Displaying image for {} channel'.format(self.imaging_channel))
            
            self.image_to_display.emit(image_resized, self.imaging_channel)
            
            if(self.imaging_channel == TRACKING):
                # Send thresholded image to display (only for tracking stream)
                self.thresh_image_to_display.emit(image_thresh)
                self.signal_working_resolution.emit(round(self.crop_width*self.working_resolution_scaling))
            
            self.timestamp_last_display = time_now

            self.get_real_display_fps()

            
        # send image to write
        time_now = time.time()
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            if camera.is_color:
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            
            print('Sending image for save')
            self.packet_image_to_write.emit(image, camera.frame_ID, self.camera.timestamp)
            
            self.fps_save_real = round(1/(time_now - self.timestamp_last_save),1)
             # Send the real display FPS to the live Controller widget.
            self.signal_fps_save.emit(self.imaging_channel, self.fps_save_real)
            
            self.counter_save = 0

            self.timestamp_last_save = time_now

        else:
            self.counter_save += 1





        

        self.handler_busy = False
        camera.image_locked = False


    # def on_new_frame_from_simulation(self,image,frame_ID = None,timestamp = None):
    #     # check whether image is a local copy or pointer, if a pointer, needs to prevent the image being modified while this function is being executed
    #     color = False

    #     self.handler_busy = True
    #     self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()
    #     # This also triggers the microcontroller_Receiever

    #     # measure real fps
    #     timestamp_now = round(time.time())
    #     if timestamp_now == self.timestamp_last:
    #         self.counter = self.counter+1
    #     else:
    #         self.timestamp_last = timestamp_now
    #         self.fps_real = self.counter
    #         self.counter = 0
    #         print('real camera fps is ' + str(self.fps_real))

    #     # crop image
    #     # image = image_processing.crop_image(camera.current_frame,self.crop_width,self.crop_height)
    #     image = np.array(np.copy(image), dtype = 'uint8')
    #     # save a copy of full-res image for saving (make sure to do a deep copy)
    #     # @@@@@@@@@

    #     # image_resized = cv2.resize(image,(round(self.crop_width*self.working_resolution_scaling), round(self.crop_height*self.working_resolution_scaling)),cv2.INTER_LINEAR)
    #     image_resized = imutils.resize(image, round(self.image_width*self.working_resolution_scaling))

        
        
    #     # Deepak: For now tracking with every image from camera 
    #     if self.track_flag and self.trackingStream:
    #         # track is a blocking operation - it needs to be
    #         # @@@ will cropping before emitting the signal lead to speedup?
    #         # print('Sending image to tracking controller...')

    #         image_thresh = 255*np.array(self.threshold_image(image_resized, color = False), dtype='uint8')

    #         cv2.imshow('Thresh image',image_thresh)
    #         cv2.waitKey(1)

    #         self.packet_image_for_tracking.emit(image_resized, image_thresh)
    #         self.timestamp_last_track = timestamp_now

    #     # send image to display
    #     time_now = time.time()
    #     if time_now - self.timestamp_last_display >= 1/self.fps_display:
    #         # print('Sending image to display...')
    #         if color:
    #             image_resized = cv2.cvtColor(image_resized,cv2.COLOR_RGB2BGR)

    #         self.image_to_display.emit(image_resized, self.trackingStream)
            
    #         self.timestamp_last_display = time_now

    #     # send image to write
    #     if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
    #         # print('Saving image...')
    #         if color:
    #             image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    #         self.packet_image_to_write.emit(image,camera.frame_ID,camera.timestamp)
    #         self.timestamp_last_save = time_now

    #     self.handler_busy = False

    # def stop(self):
    #     pass
    #     # self.camera.stop()


class LiveController(QObject):

    def __init__(self,camera,microcontroller):
        QObject.__init__(self)
        self.camera = camera
        self.microcontroller = microcontroller
        self.microscope_mode = None
        self.trigger_mode = TriggerMode.SOFTWARE # @@@ change to None
        self.mode = None
        self.is_live = False
        self.was_live_before_autofocus = False
        self.was_live_before_multipoint = False

        self.fps_software_trigger = 1;
        self.timer_software_trigger_interval = (1/self.fps_software_trigger)*1000

        self.timer_software_trigger = QTimer()
        self.timer_software_trigger.setInterval(self.timer_software_trigger_interval)
        self.timer_software_trigger.timeout.connect(self.trigger_acquisition_software)

        self.trigger_ID = -1

        self.fps_real = 0
        self.counter = 0
        self.timestamp_last = 0

        self.exposure_time_bfdf_preset = None
        self.exposure_time_fl_preset = None
        self.exposure_time_fl_preview_preset = None
        self.analog_gain_bfdf_preset = None
        self.analog_gain_fl_preset = None
        self.analog_gain_fl_preview_preset = None

    # illumination control
    def turn_on_illumination(self):
        if self.mode == MicroscopeMode.BFDF:
            self.microcontroller.toggle_LED(1)
        else:
            self.microcontroller.toggle_laser(1)

    def turn_off_illumination(self):
        if self.mode == MicroscopeMode.BFDF:
            self.microcontroller.toggle_LED(0)
        else:
            self.microcontroller.toggle_laser(0)

    def start_live(self):
        self.is_live = True
        self.camera.start_streaming()
        if self.trigger_mode == TriggerMode.SOFTWARE:
            self._start_software_triggerred_acquisition()

    def stop_live(self):
        if self.is_live:
            self.is_live = False
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            self.camera.stop_streaming()
            self.turn_off_illumination()

    # software trigger related
    def trigger_acquisition_software(self):
        self.turn_on_illumination()
        self.trigger_ID = self.trigger_ID + 1
        self.camera.send_trigger()
        # measure real fps
        timestamp_now = round(time.time())
        if timestamp_now == self.timestamp_last:
            self.counter = self.counter+1
        else:
            self.timestamp_last = timestamp_now
            self.fps_real = self.counter
            self.counter = 0
            # print('real trigger fps is ' + str(self.fps_real))

    def _start_software_triggerred_acquisition(self):
        self.timer_software_trigger.start()

    def _set_software_trigger_fps(self,fps_software_trigger):
        self.fps_software_trigger = fps_software_trigger
        self.timer_software_trigger_interval = (1/self.fps_software_trigger)*1000
        self.timer_software_trigger.setInterval(self.timer_software_trigger_interval)

    def _stop_software_triggerred_acquisition(self):
        self.timer_software_trigger.stop()

    # trigger mode and settings
    def set_trigger_mode(self, mode):
        if mode == TriggerMode.SOFTWARE:
            self.camera.set_software_triggered_acquisition()
            if self.is_live:
                self._start_software_triggerred_acquisition()
        if mode == TriggerMode.HARDWARE:
            print('hardware trigger to be added')
            #self.camera.set_hardware_triggered_acquisition()
        if mode == TriggerMode.CONTINUOUS: 
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_software_triggerred_acquisition()
            if mode == TriggerMode.HARDWARE:
                pass #@@@ to be implemented
            self.camera.set_continuous_acquisition()
        self.trigger_mode = mode

    def set_trigger_fps(self,fps):
        if self.trigger_mode == TriggerMode.SOFTWARE:
            self._set_software_trigger_fps(fps)
    
    # set microscope mode
    # @@@ to do: change softwareTriggerGenerator to TriggerGeneratror
    def set_microscope_mode(self,mode):
        print("setting microscope mode to " + mode)
        
        # temporarily stop live while changing mode
        if self.is_live is True:
            self.timer_software_trigger.stop()
            self.turn_off_illumination()
        
        self.mode = mode
        if self.mode == MicroscopeMode.BFDF:
            self.camera.set_exposure_time(self.exposure_time_bfdf_preset)
            self.camera.set_analog_gain(self.analog_gain_bfdf_preset)
        elif self.mode == MicroscopeMode.FLUORESCENCE:
            self.camera.set_exposure_time(self.exposure_time_fl_preset)
            self.camera.set_analog_gain(self.analog_gain_fl_preset)
        elif self.mode == MicroscopeMode.FLUORESCENCE_PREVIEW:
            self.camera.set_exposure_time(self.exposure_time_fl_preview_preset)
            self.camera.set_analog_gain(self.analog_gain_fl_preview_preset)

        # restart live 
        if self.is_live is True:
            self.turn_on_illumination()
            self.timer_software_trigger.start()

    def get_trigger_mode(self):
        return self.trigger_mode

    def set_exposure_time_bfdf_preset(self,exposure_time):
        self.exposure_time_bfdf_preset = exposure_time
    def set_exposure_time_fl_preset(self,exposure_time):
        self.exposure_time_fl_preset = exposure_time
    def set_exposure_time_fl_preview_preset(self,exposure_time):
        self.exposure_time_fl_preview_preset = exposure_time
    def set_analog_gain_bfdf_preset(self,analog_gain):
        self.analog_gain_bfdf_preset = analog_gain
    def set_analog_gain_fl_preset(self,analog_gain):
        self.analog_gain_fl_preset = analog_gain
    def set_analog_gain_fl_preview_preset(self,analog_gain):
        self.analog_gain_fl_preview_preset = analog_gain

    # slot
    def on_new_frame(self):
        if self.fps_software_trigger <= 5:
            self.turn_off_illumination()

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0

    def move_x(self,delta):
        self.microcontroller.move_x(delta)
        self.x_pos = self.x_pos + delta
        self.xPos.emit(self.x_pos)

    def move_y(self,delta):
        self.microcontroller.move_y(delta)
        self.y_pos = self.y_pos + delta
        self.yPos.emit(self.y_pos)

    def move_z(self,delta):
        self.microcontroller.move_z(delta)
        self.z_pos = self.z_pos + delta
        self.zPos.emit(self.z_pos*1000)

        

class ImageDisplay(QObject):

    image_to_display = Signal(np.ndarray, str)

    def __init__(self):
        QObject.__init__(self)
        self.queue = Queue(10) # max 10 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()        
        
    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image, frame_ID, timestamp, imaging_channel] = self.queue.get(timeout=0.1)
           
                self.image_lock.acquire(True)
                # Send image and imaging_channel
                self.image_to_display.emit(image, imaging_channel)
               
                self.image_lock.release()
                self.queue.task_done()
            except:
                # print("Exception:", sys.exc_info()[0])
                # print('Not sending image to display window')
                pass

    # def enqueue(self,image,frame_ID,timestamp):
    def enqueue(self,image, trackingStream = False):
        try:
            # print('In image display queue')
            self.queue.put_nowait([image, None, None, imaging_channel])
            # when using self.queue.put(str_) instead of try + nowait, program can be slowed down despite multithreading because of the block and the GIL
        
        except:
            pass
            # print('imageDisplay queue is full, image discarded')

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()

    # def __del__(self):
        # self.wait()

# from gravity machine
class ImageDisplayWindow(QMainWindow):

    roi_bbox = Signal(np.ndarray)

    def __init__(self, window_title='', DrawCrossHairs = False, rotate_image_angle = 0, flip_image = None):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()

        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.view = self.graphics_widget.addViewBox()
        
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        
        ## Create image item
        self.graphics_widget.img = pg.ImageItem(border='w')
        self.graphics_widget.view.addItem(self.graphics_widget.img)

        self.image_width = None
        self.image_height = None

        ## Create ROI
        self.roi_pos = (0.5,0.5)
        self.roi_size = (500,500)
        self.ROI = pg.ROI(self.roi_pos, self.roi_size, scaleSnap=True, translateSnap=True)
        self.ROI.setZValue(10)
        self.ROI.addScaleHandle((0,0), (1,1))
        self.ROI.addScaleHandle((1,1), (0,0))
        self.graphics_widget.view.addItem(self.ROI)
        self.ROI.hide()
        self.ROI.sigRegionChanged.connect(self.updateROI)
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

        ## Variables for annotating images
        self.DrawRect = False
        self.ptRect1 = None
        self.ptRect2 = None

        self.DrawCirc = False
        self.centroid = None

        self.DrawCrossHairs = DrawCrossHairs

        self.image_offset = np.array([0, 0])

        # Image rotations and flipping
        self.rotate_image_angle = rotate_image_angle
        self.flip_image = flip_image

        layout = QGridLayout()
        layout.addWidget(self.graphics_widget, 0, 0) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

    def display_image(self,image, imaging_channel = TRACKING):
        
        image = np.copy(image) # Avoid overwriting the source image
        
        if(imaging_channel == TRACKING):

            self.image_height, self.image_width = image_processing.get_image_height_width(image)


            if(self.DrawRect):
                cv2.rectangle(image, self.ptRect1, self.ptRect2,(0,0,0) , 2) #cv2.rectangle(img, (20,20), (300,300),(0,0,255) , 2)#
                self.DrawRect=False

            if(self.DrawCirc):
                cv2.circle(image,(self.centroid[0],self.centroid[1]), 20, (255,0,0), 2)
                self.DrawCirc=False

            if(self.DrawCrossHairs):
                # Only need to do this if the image size changes
                self.update_image_center_width(image)

                self.draw_crosshairs()


                cv2.line(image, self.horLine_pt1, self.horLine_pt2, (255,255,255), thickness=1, lineType=8, shift=0) 
                cv2.line(image, self.verLine_pt1, self.verLine_pt2, (255,255,255), thickness=1, lineType=8, shift=0) 
        
        if(self.rotate_image_angle != 0):
            '''
                # ROTATE_90_CLOCKWISE
                # ROTATE_90_COUNTERCLOCKWISE
            '''
            if(self.rotate_image_angle == 90):
                image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
            elif(self.rotate_image_angle == -90):
                image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)

        if(self.flip_image is not None):
            '''
                flipcode = 0: flip vertically
                flipcode > 0: flip horizontally
                flipcode < 0: flip vertically and horizontally
            '''
            if(self.flip_image == 'Vertical'):
                image = cv2.flip(image, 0)
            elif(self.flip_image == 'Horizontal'):
                image = cv2.flip(image, 1)
            elif(self.flip_image == 'Both'):
                image = cv2.flip(image, -1)


     


        self.graphics_widget.img.setImage(image,autoLevels=False)
        # print('In ImageDisplayWindow display image')
    
    
    def draw_rectangle(self, pts):
        # Connected to Signal from Tracking object
        self.DrawRect=True
        self.ptRect1=(pts[0][0],pts[0][1])
        self.ptRect2=(pts[1][0],pts[1][1])

    def draw_circle(self, centroid):
        # Connected to Signal from Tracking object
        self.DrawCirc=True
        self.centroid=(centroid[0],centroid[1])
        
    def draw_crosshairs(self):
        # Connected to Signal from Tracking object
        cross_length = round(self.image_width/20)

        self.horLine_pt1 = (int(self.tracking_center[0] - cross_length/2), int(self.tracking_center[1]))
        self.horLine_pt2 = (int(self.tracking_center[0] + cross_length/2), int(self.tracking_center[1]))

        self.verLine_pt1 = (int(self.tracking_center[0]), int(self.tracking_center[1] - cross_length/2))
        self.verLine_pt2 = (int(self.tracking_center[0]), int(self.tracking_center[1] + cross_length/2))


    def update_image_center_width(self,image):
        self.image_center, self.image_width = image_processing.get_image_center_width(image)
        self.tracking_center = self.image_center + self.image_offset

    def update_image_offset(self, new_image_offset):
        self.image_offset = new_image_offset
        print('ROI pos: {}'.format(self.roi_pos))
        print('ROI size: {}'.format(self.roi_size))

    def updateROI(self):
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

    def toggle_ROI_selector(self, flag):
        if(flag == True):
            self.ROI.show()
        else:
            self.ROI.hide()

    def send_bbox(self):
        self.updateROI()
        width = self.roi_size[0]
        height = self.roi_size[1]
        xmin = max(0, self.roi_pos[0])
        ymin = max(0, self.image_height - height - self.roi_pos[1])
        # print('Bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))

        self.roi_bbox.emit(np.array([xmin, ymin, width, height]))
        # print('Sent bbox from ImageDisplay: {}'.format([xmin, ymin, width, height]))


class AutoFocusController(QObject):
    z_pos = Signal(float)
    autofocusFinished = Signal()
    image_to_display = Signal(np.ndarray)

    def __init__(self,camera,navigationController,liveController):
        QObject.__init__(self)
        self.camera = camera
        self.navigationController = navigationController
        self.liveController = liveController
        self.N = None
        self.deltaZ = None
        self.crop_width = AF.CROP_WIDTH
        self.crop_height = AF.CROP_HEIGHT

    def set_N(self,N):
        self.N = N

    def set_deltaZ(self,deltaZ_um):
        self.deltaZ = deltaZ_um/1000

    def set_crop(self,crop_width,height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def autofocus(self):

        # stop live
        if self.liveController.is_live:
            self.liveController.was_live_before_autofocus = True
            self.liveController.stop_live()

        # temporarily disable call back -> image does not go through streamHandler
        if self.camera.callback_is_enabled:
            self.camera.callback_was_enabled_before_autofocus = True
            self.camera.stop_streaming()
            self.camera.disable_callback()
            self.camera.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        
        # @@@ to add: increase gain, decrease exposure time
        # @@@ can move the execution into a thread
        focus_measure_vs_z = [0]*self.N
        focus_measure_max = 0

        z_af_offset = self.deltaZ*round(self.N/2)
        self.navigationController.move_z(-z_af_offset)

        steps_moved = 0
        for i in range(self.N):
            self.navigationController.move_z(self.deltaZ)
            steps_moved = steps_moved + 1
            self.liveController.turn_on_illumination()
            self.camera.send_trigger()
            image = self.camera.read_frame()
            self.liveController.turn_off_illumination()
            image = utils.crop_image(image,self.crop_width,self.crop_height)
            self.image_to_display.emit(image)
            QApplication.processEvents()
            timestamp_0 = time.time() # @@@ to remove
            focus_measure = utils.calculate_focus_measure(image)
            timestamp_1 = time.time() # @@@ to remove
            print('             calculating focus measure took ' + str(timestamp_1-timestamp_0) + ' second')
            focus_measure_vs_z[i] = focus_measure
            print(i,focus_measure)
            focus_measure_max = max(focus_measure, focus_measure_max)
            if focus_measure < focus_measure_max*AF.STOP_THRESHOLD:
                break

        idx_in_focus = focus_measure_vs_z.index(max(focus_measure_vs_z))
        self.navigationController.move_z((idx_in_focus-steps_moved)*self.deltaZ)
        if idx_in_focus == 0:
            print('moved to the bottom end of the AF range')
        if idx_in_focus == self.N-1:
            print('moved to the top end of the AF range')

        if self.camera.callback_was_enabled_before_autofocus:
            self.camera.stop_streaming()
            self.camera.enable_callback()
            self.camera.start_streaming()
            self.camera.callback_was_enabled_before_autofocus = False

        if self.liveController.was_live_before_autofocus:
            self.liveController.start_live()
            self.liveController.was_live = False
        
        print('autofocus finished')
        self.autofocusFinished.emit()

class MultiPointController(QObject):

    acquisitionFinished = Signal()
    image_to_display = Signal(np.ndarray)

    x_pos = Signal(float)
    y_pos = Signal(float)
    z_pos = Signal(float)

    def __init__(self,camera,navigationController,liveController,autofocusController):
        QObject.__init__(self)

        self.camera = camera
        self.navigationController = navigationController
        self.liveController = liveController
        self.autofocusController = autofocusController
        self.NX = 1
        self.NY = 1
        self.NZ = 1
        self.Nt = 1
        self.deltaX = Acquisition.DX
        self.deltaY = Acquisition.DY
        self.deltaZ = Acquisition.DZ/1000
        self.deltat = 0
        self.do_bfdf = False
        self.do_fluorescence = False
        self.do_autofocus = False
        self.crop_width = Acquisition.CROP_WIDTH
        self.crop_height = Acquisition.CROP_HEIGHT
        self.display_resolution_scaling = Acquisition.IMAGE_DISPLAY_SCALING_FACTOR
        self.counter = 0
        self.experiment_ID = None
        self.base_path = None

    def set_NX(self,N):
        self.NX = N
    def set_NY(self,N):
        self.NY = N
    def set_NZ(self,N):
        self.NZ = N
    def set_Nt(self,N):
        self.Nt = N
    def set_deltaX(self,delta):
        self.deltaX = delta
    def set_deltaY(self,delta):
        self.deltaY = delta
    def set_deltaZ(self,delta_um):
        self.deltaZ = delta_um/1000
    def set_deltat(self,delta):
        self.deltat = delta
    def set_bfdf_flag(self,flag):
        self.do_bfdf = flag
    def set_fluorescence_flag(self,flag):
        self.do_fluorescence = flag
    def set_af_flag(self,flag):
        self.do_autofocus = flag

    def set_crop(self,crop_width,crop_height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_base_path(self,path):
        self.base_path = path

    def start_new_experiment(self,experiment_ID): # @@@ to do: change name to prepare_folder_for_new_experiment
        # generate unique experiment ID
        self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%y-%m-%d %H-%M-%S.%f')
        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        except:
            pass
        
    def run_acquisition(self): # @@@ to do: change name to run_experiment
        print('start multipoint')
        print(str(self.Nt) + '_' + str(self.NX) + '_' + str(self.NY) + '_' + str(self.NZ))

        self.time_point = 0
        self.single_acquisition_in_progress = False
        self.acquisitionTimer = QTimer()
        self.acquisitionTimer.setInterval(self.deltat*1000)
        self.acquisitionTimer.timeout.connect(self._on_acquisitionTimer_timeout)
        self.acquisitionTimer.start()
        self.acquisitionTimer.timeout.emit() # trigger the first acquisition

    def _on_acquisitionTimer_timeout(self):
        # check if the last single acquisition is ongoing
        if self.single_acquisition_in_progress is True:
            # skip time point if self.deltat is nonzero
            if self.deltat > 0.1: # @@@ to do: make this more elegant - note that both self.deltat is not 0 and self.deltat is not .0 don't work
                self.time_point = self.time_point + 1
                # stop the timer if number of time points is equal to Nt (despite some time points may have been skipped)
                if self.time_point >= self.Nt:
                    self.acquisitionTimer.stop()
                else:
                    print('the last acquisition has not completed, skip time point ' + str(self.time_point))
            return
        # if not, run single acquisition
        self._run_single_acquisition()

    def _run_single_acquisition(self):           
        self.single_acquisition_in_progress = True
        self.FOV_counter = 0

        print('multipoint acquisition - time point ' + str(self.time_point))

        # stop live
        if self.liveController.is_live:
            self.liveController.was_live_before_multipoint = True
            self.liveController.stop_live() # @@@ to do: also uncheck the live button

        # disable callback
        if self.camera.callback_is_enabled:
            self.camera.callback_was_enabled_before_multipoint = True
            self.camera.stop_streaming()
            self.camera.disable_callback()
            self.camera.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        
        # do the multipoint acquisition

        # for each time point, create a new folder
        current_path = os.path.join(self.base_path,self.experiment_ID,str(self.time_point))
        os.mkdir(current_path)

        # along y
        for i in range(self.NY):

            # along x
            for j in range(self.NX):

                # z-stack
                for k in range(self.NZ):

                    # perform AF only if when not taking z stack
                    if (self.NZ == 1) and (self.do_autofocus) and (self.FOV_counter%Acquisition.NUMBER_OF_FOVS_PER_AF==0):
                        self.autofocusController.autofocus()

                    file_ID = str(i) + '_' + str(j) + '_' + str(k)

                    # take bf
                    if self.do_bfdf:
                        self.liveController.set_microscope_mode(MicroscopeMode.BFDF)
                        self.liveController.turn_on_illumination()
                        print('take bf image')
                        self.camera.send_trigger() 
                        image = self.camera.read_frame()
                        self.liveController.turn_off_illumination()
                        image = utils.crop_image(image,self.crop_width,self.crop_height)
                        saving_path = os.path.join(current_path, file_ID + '_bf' + '.' + Acquisition.IMAGE_FORMAT)
                        # self.image_to_display.emit(cv2.resize(image,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                        self.image_to_display.emit(utils.crop_image(image,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)))
                        if self.camera.is_color:
                            image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                        cv2.imwrite(saving_path,image)
                        QApplication.processEvents()

                    # take fluorescence
                    if self.do_fluorescence:
                        self.liveController.set_microscope_mode(MicroscopeMode.FLUORESCENCE)
                        self.liveController.turn_on_illumination()
                        self.camera.send_trigger()
                        image = self.camera.read_frame()
                        print('take fluorescence image')
                        self.liveController.turn_off_illumination()
                        image = utils.crop_image(image,self.crop_width,self.crop_height)
                        saving_path = os.path.join(current_path, file_ID + '_fluorescence' + '.' + Acquisition.IMAGE_FORMAT)
                        self.image_to_display.emit(utils.crop_image(image,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)))
                        # self.image_to_display.emit(cv2.resize(image,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                        if self.camera.is_color:
                            image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)                        
                        cv2.imwrite(saving_path,image)                        
                        QApplication.processEvents()
                    
                    if self.do_bfdf is not True and self.do_fluorescence is not True:
                        QApplication.processEvents()

                    # move z
                    if k < self.NZ - 1:
                        self.navigationController.move_z(self.deltaZ)
                
                # move z back
                self.navigationController.move_z(-self.deltaZ*(self.NZ-1))

                # update FOV counter
                self.FOV_counter = self.FOV_counter + 1

                # move x
                if j < self.NX - 1:
                    self.navigationController.move_x(self.deltaX)

            # move x back
            self.navigationController.move_x(-self.deltaX*(self.NX-1))

            # move y
            if i < self.NY - 1:
                self.navigationController.move_y(self.deltaY)

        # move y back
        self.navigationController.move_y(-self.deltaY*(self.NY-1))
                        
        # re-enable callback
        if self.camera.callback_was_enabled_before_multipoint:
            self.camera.stop_streaming()
            self.camera.enable_callback()
            self.camera.start_streaming()
            self.camera.callback_was_enabled_before_multipoint = False
        
        if self.liveController.was_live_before_multipoint:
            self.liveController.start_live()
            # emit acquisitionFinished signal
            self.acquisitionFinished.emit()
        
        # update time_point for the next scheduled single acquisition (if any)
        self.time_point = self.time_point + 1

        if self.time_point >= self.Nt:
            print('Multipoint acquisition finished')
            if self.acquisitionTimer.isActive():
                self.acquisitionTimer.stop()
            self.acquisitionFinished.emit()

        self.single_acquisition_in_progress = False




class DishScanController(QObject):
    acquisitionFinished = Signal()
    image_to_display = Signal(np.ndarray)

    x_pos = Signal(float)
    y_pos = Signal(float)
    z_pos = Signal(float)

    def __init__(self,camera,navigationController,liveController,autofocusController):
        QObject.__init__(self)

        self.camera = camera
        self.navigationController = navigationController
        self.liveController = liveController
        self.autofocusController = autofocusController

        self.NX = 1  # this is the number of steps to be taken for each well in x direction
        self.NY = 1  # this is the number of steps to be taken for each well in y direction
        self.NZ = 1
        self.Nt = 1
        self.deltaX = DishAcquisition.WIDTHX
        self.deltaY = DishAcquisition.WIDTHY
        self.deltaZ = DishAcquisition.DZ/1000  #why divided by 1000, check, to convert it into microns
        self.deltat = 0
        self.FOVoverlap = DishAcquisition.OVERLAP

        self.do_subsill = False
        self.do_offnot = False
        self.do_autofocus = False
        self.do_scansave = False

        self.crop_width = DishAcquisition.CROP_WIDTH
        self.crop_height = DishAcquisition.CROP_HEIGHT
        self.display_resolution_scaling = DishAcquisition.IMAGE_DISPLAY_SCALING_FACTOR
        self.counter = 0
        self.experiment_ID = None
        self.base_path = None
        self.home_coord = None ##### change this based on how you use the home coordinate value
        self.dish_template = None  #initial template is none
        self.well_ID_list = []  #initial setup as empty list for wells
        self.well_labels = []
        self.originPOS = [] #a list for coordinates for the origin of all the wells
        self.posJump = [] #the jumps in position required to reach the origins of wells after starting from home position

        #get scale factors for images being captures through camera in pizels per mm
        self.scale_5x = TemplateDef.scale_5x_pxmm
        self.scale_2x = TemplateDef.scale_2x_pxmm
        self.scale_used = self.scale_5x  #change it based on the lens used
        self.magnification = 5  #change to the magnification of the lens being used

        self.frameSizeX = 0.5*DishAcquisition.WIDTHX/self.magnification
        self.frameSizeY = 0.5*DishAcquisition.WIDTHY/self.magnification

        # 9 pelco 50 mm dish template data, all values in cms
        self.pelco_50_sz = TemplateDef.pelco_50_sz_mm  # in mm
        self.pelco_50_sep = TemplateDef.pelco_50_sep_mm  # in mm #separation between walls of adjacent dishes


    def set_subsill_flag(self,flag):
        self.do_subsill = flag
    def set_offnot_flag(self,flag):
        self.do_offnot = flag
    def set_af_flag(self,flag):
        self.do_autofocus = flag
    def set_scansave_flag(self,flag):
        self.do_scansave = flag

    def set_crop(self,crop_width,crop_height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_base_path(self,path):
        self.base_path = path

# use the marking circle on the template to start the acquisition for the Pelco 9 50mm or for the 6 well plate.
    def set_home(self,home):
        self.home_coord = home
        print(self.home_coord)

    def set_period(self,period, loops):
        self.deltat = period
        self.Nt = loops
        print(self.deltat, "seconds", self.Nt, "loops")

    def set_template(self,template, ID_list):
        self.dish_template = template
        self.well_ID_list = ID_list
        print(self.dish_template, self.well_ID_list)
        if (self.dish_template == 0) :  #repeat this for all the scan templates
            for ii in self.well_ID_list:
                self.well_labels.append([(ii-1)%3, (ii-1)//3]) 
            print(self.well_labels)

            self.NX = int(40/(self.frameSizeX*self.FOVoverlap))-int(1/self.FOVoverlap-1)
            self.NY = int(40/(self.frameSizeY*self.FOVoverlap))-int(1/self.FOVoverlap-1)
            self.NZ = 1
            self.deltaX = self.frameSizeX*self.FOVoverlap  #in mm
            self.deltaY = self.frameSizeY*self.FOVoverlap  #in mm
            self.deltaZ = DishAcquisition.DZ/1000  #why divided by 1000, check, to convert it into microns
            originPOS_bank = [[2.0,2.0], [8.5,2.0], [15.0,2.0], [2.0, 8.5], [8.5, 8.5], [15.0, 8.5], [2.0, 15.0], [8.5,15.0], [15.0,15.0]]
            
            for ii in self.well_ID_list:
                self.originPOS.append(originPOS_bank[ii-1]) 

            self.posJump.append(self.originPOS[0])
            for ii in range(1,len(self.well_ID_list)):
                self.posJump.append([self.originPOS[ii][0] - self.originPOS[ii-1][0], self.originPOS[ii][1] - self.originPOS[ii-1][1]])

            print(self.originPOS, self.posJump)

        if (self.dish_template == 1) :  #repeat this for all the scan templates
            for ii in self.well_ID_list:
                self.well_labels.append([(ii-1)%3, (ii-1)//3])
            print(self.well_labels)
            # self.NX = N
            # self.NY = N
            # self.NZ = N
            # self.Nt = N
            # self.deltaX = delta
            # self.deltaY = delta
            # self.deltaZ= delta


    def start_new_experiment(self,experiment_ID): # @@@ to do: change name to prepare_folder_for_new_experiment
        # generate unique experiment ID
        self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%S.%f')
        print(self.experiment_ID)
        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        except:
            pass

    def run_acquisition(self): # @@@ to do: change name to run_experiment
        print('start dishscan')
        print(str(self.Nt) + '_' + str(self.NX) + '_' + str(self.NY) + '_' + str(self.NZ))  #change the display tag based on the combo settings

        self.time_point = 0
        self.single_acquisition_in_progress = False
        self.acquisitionTimer = QTimer()
        self.acquisitionTimer.setInterval(self.deltat*1000)  # takes input in milliseconds
        self.acquisitionTimer.timeout.connect(self._on_acquisitionTimer_timeout)                #how are multiple time points run using this?
        self.acquisitionTimer.start()
        self.acquisitionTimer.timeout.emit() # trigger the first acquisition

    def _on_acquisitionTimer_timeout(self):
        # check if the last single acquisition is ongoing
        if self.single_acquisition_in_progress is True:
            # skip time point if self.deltat is nonzero
            if self.deltat > 0.1: # @@@ to do: make this more elegant - note that both self.deltat is not 0 and self.deltat is not .0 don't work
                self.time_point = self.time_point + 1
                # stop the timer if number of time points is equal to Nt (despite some time points may have been skipped)
                if self.time_point >= self.Nt:
                    self.acquisitionTimer.stop()
                else:
                    print('the last acquisition has not completed, skip time point ' + str(self.time_point))
            return
        # if not, run single acquisition
        self._run_single_acquisition()

    def _run_single_acquisition(self):
        self.is_animal = False
        self.animal_saved = False
        self.cent_coord = []
        self.single_acquisition_in_progress = True
        self.FOV_counter = 0
        self.shiftX = 0
        self.shiftY = 0

        print('dish scanning - time point' + str(self.time_point))

        # stop live
        if self.liveController.is_live:
            self.liveController.was_live_before_multipoint = True
            self.liveController.stop_live() # @@@ to do: also uncheck the live button

        # disable callback
        if self.camera.callback_is_enabled:
            self.camera.callback_was_enabled_before_multipoint = True
            self.camera.stop_streaming()
            self.camera.disable_callback()
            self.camera.start_streaming() # @@@ to do: absorb stop/start streaming into enable/disable callback - add a flag is_streaming to the camera class
        
        # do the multipoint acquisition

        # for each time point, create a new folder
        time_path = os.path.join(self.base_path,self.experiment_ID,str(self.time_point))
        os.mkdir(time_path)

        # shift to get to the camera on the center of the frame from the corner
        self.navigationController.move_x(self.frameSizeX/2)
        self.navigationController.move_y(self.frameSizeX/2)

        # for parsing across well ids
        ctr = 0
        for ww in self.well_labels:
            #for each well create a new folder
            current_path = os.path.join(time_path,"well_"+str(ww[0]+1+ww[1]*3))
            os.mkdir(current_path)
            print(current_path,ww)
            
            #refresh the list to store the values of coordinates of tracked animals based on ww, i, j, k
            coord_list = []
                        
            # write a function to go to the top left corner of the new well and then do a regular scan acquisition using NX and NY
            self.navigationController.move_x(self.posJump[ctr][0])
            self.navigationController.move_y(self.posJump[ctr][1])

            # along y
            for i in range(self.NY):

                # along x
                for j in range(self.NX):

                    # z-stack
                    for k in range(self.NZ):
    
                        # perform AF only if when not taking z stack
                        if (self.NZ == 1) and (self.do_autofocus) and (self.FOV_counter%Acquisition.NUMBER_OF_FOVS_PER_AF==0):
                            self.autofocusController.autofocus()
    
                        file_ID = str(i) + '_' + str(j) + '_' + str(k)
    
                        # take bf
                        if self.do_subsill:
                            self.liveController.set_microscope_mode(MicroscopeMode.BFDF)  #connect microcontroller to control the substrate illumination
                            self.liveController.turn_on_illumination()
                            #print('take bf image')
                            self.camera.send_trigger()
                            image = self.camera.read_frame()
                            self.liveController.turn_off_illumination()
                            image = utils.crop_image(image,self.crop_width,self.crop_height)
                            saving_path = os.path.join(current_path, file_ID + '_subsill' + '.' + DishAcquisition.IMAGE_FORMAT)
                            # self.image_to_display.emit(cv2.resize(image,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                            self.image_to_display.emit(utils.crop_image(image,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)))
                            if self.camera.is_color:
                                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                            if self.do_scansave is False:
                                cv2.imwrite(saving_path,image)
                                print('saving image')
                            else:
                                t = time.time()
                                self.is_animal, self.cent_coord = detection.detect_animal(image)
                                #check if the animal has already been scanned, leave if already scanned
                                self.animal_saved = detection.check_saved(self.cent_coord)
                                # do stuff
                                elapsed = time.time() - t
                                print(elapsed, self.cent_coord)
                                if (self.is_animal) and (not self.animal_saved):
                                    #save halt location and get displacements necessary to get the center the animal in the frame  - move to the animal
                                    self.shiftX = (int(self.cent_coord[0])-965)/self.scale_used
                                    self.shiftY = (int(self.cent_coord[1])-965)/self.scale_used
                                    self.navigationController.move_x(self.shiftX)
                                    self.navigationController.move_y(self.shiftY)
                                    
                                    
                                    self.liveController.turn_on_illumination()
                                    print('taking image')
                                    self.camera.send_trigger()
                                    image = self.camera.read_frame()
                                    self.liveController.turn_off_illumination()
                                    image = utils.crop_image(image,self.crop_width,self.crop_height)
                                    cv2.imwrite(saving_path,image)

                                    #save the position of the animal relative to the center of the dish in a data array for the well
                                    
                                    
                                    #return back to the original halt position
                                    self.navigationController.move_y(-self.shiftY)
                                    self.navigationController.move_x(-self.shiftX)
                                else:
                                    print('not taking image')
                                    #continue scanning
                                    
                            QApplication.processEvents()
                        
                        if self.do_subsill is not True:
                            QApplication.processEvents()
    
                        # move z
                        if k < self.NZ - 1:
                            self.navigationController.move_z(self.deltaZ)
                    
                    # move z back
                    self.navigationController.move_z(-self.deltaZ*(self.NZ-1))
    
                    # update FOV counter
                    self.FOV_counter = self.FOV_counter + 1
    
                    # move x
                    if j < self.NX - 1:
                        self.navigationController.move_x(self.deltaX)
    
                # move x back
                self.navigationController.move_x(-self.deltaX*(self.NX-1))
    
                # move y
                if i < self.NY - 1:
                    self.navigationController.move_y(self.deltaY)
    
            # move y back
            self.navigationController.move_y(-self.deltaY*(self.NY-1))
            
            #save the data for this well in a txt file
            coord_array = np.array(coord_list)
            
            #reset FOV counter for a new well 
            self.FOV_counter = 0
            
            ctr = ctr +1 

        #reach back near home from where scanning was started
        self.navigationController.move_x(-self.originPOS[ctr-1][0])
        self.navigationController.move_y(-self.originPOS[ctr-1][1])

        #reverse the shift from center of the frame to the corner
        self.navigationController.move_x(-self.frameSizeX/2)
        self.navigationController.move_y(-self.frameSizeX/2)

        #reach the home position from where the scanning was started
        
        
        # re-enable callback
        if self.camera.callback_was_enabled_before_multipoint:
            self.camera.stop_streaming()
            self.camera.enable_callback()
            self.camera.start_streaming()
            self.camera.callback_was_enabled_before_multipoint = False
        
        if self.liveController.was_live_before_multipoint:
            self.liveController.start_live()
            # emit acquisitionFinished signal
            self.acquisitionFinished.emit()
        
        # update time_point for the next scheduled single acquisition (if any)
        self.time_point = self.time_point + 1

        if self.time_point >= self.Nt:
            print('Dish scanning finished')
            if self.acquisitionTimer.isActive():
                self.acquisitionTimer.stop()
            self.acquisitionFinished.emit()

        self.single_acquisition_in_progress = False