# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

import pyqtgraph as pg
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel
import control.utils.dockareaStyle as dstyle


import numpy as np
from collections import deque

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class CameraSettingsWidget(QFrame):

	def __init__(self, camera, liveController, main=None, *args, **kwargs):

		super().__init__(*args, **kwargs)
		self.camera = camera
		self.liveController = liveController
		# add components to self.grid
		self.add_components()        
		# set frame style
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		# add buttons and input fields
		self.entry_exposureTime = QDoubleSpinBox()
		self.entry_exposureTime.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN) 
		self.entry_exposureTime.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX) 
		self.entry_exposureTime.setSingleStep(1)
		self.entry_exposureTime.setValue(20)
		# self.camera.set_exposure_time(20)

		self.entry_analogGain = QDoubleSpinBox()
		self.entry_analogGain.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain.setValue(10)
		self.camera.set_analog_gain(10)

		self.entry_exposureTime_Preset = QDoubleSpinBox()
		self.entry_exposureTime_Preset.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN)
		self.entry_exposureTime_Preset.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX)
		self.entry_exposureTime_Preset.setSingleStep(1)
		self.entry_exposureTime_Preset.setValue(20)
		# self.liveController.set_exposure_time_preset(20)

		self.entry_analogGain_Preset = QDoubleSpinBox()
		self.entry_analogGain_Preset.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain_Preset.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain_Preset.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain_Preset.setValue(0)
		# self.liveController.set_analog_gain_preset(0)


		self.btn_Preset = QPushButton("Preset")
		self.btn_Preset.setDefault(False)
   

		# connection
		self.btn_Preset.clicked.connect(self.load_preset)
		
		self.entry_exposureTime.valueChanged.connect(self.camera.set_exposure_time)
		self.entry_analogGain.valueChanged.connect(self.camera.set_analog_gain)
		self.entry_exposureTime_Preset.valueChanged.connect(self.liveController.set_exposure_time_bfdf_preset)
		self.entry_analogGain_Preset.valueChanged.connect(self.liveController.set_analog_gain_bfdf_preset)
		
		# layout
		grid_ctrl = QGridLayout()
		grid_ctrl.addWidget(QLabel('Exposure Time (ms)'), 0,0)
		grid_ctrl.addWidget(self.entry_exposureTime, 0,1)
		grid_ctrl.addWidget(QLabel('Analog Gain'), 1,0)
		grid_ctrl.addWidget(self.entry_analogGain, 1,1)

		grid_ctrl_preset = QGridLayout()
		grid_ctrl_preset.addWidget(self.entry_exposureTime_Preset, 0,0)
		grid_ctrl_preset.addWidget(self.entry_analogGain_Preset, 0,1)
		grid_ctrl_preset.addWidget(self.btn_Preset, 0,2)
	  

		self.grid = QGridLayout()
		self.grid.addLayout(grid_ctrl,0,0)
		self.grid.addLayout(grid_ctrl_preset,1,0)

		self.setLayout(self.grid)

	def load_preset(self):
		self.entry_exposureTime.setValue(self.entry_exposureTime_Preset.value())
		self.entry_exposureTime.repaint() # update doesn't work
		self.entry_analogGain.setValue(self.entry_analogGain_Preset.value())
		self.entry_analogGain.repaint()


class LiveControlWidget(QGroupBox):
	'''
	Widget controls salient microscopy parameters such as:
		- Live Button
		- Checkboxes to choose active image-streams.
		- Objective
		- Display resolution slider
	'''
	objective_signal = Signal(str) # Pixel size based on calibration image
	resolution_scaling_signal = Signal(int)
	show_window = Signal(bool)

	def __init__(self, streamHandler, liveController, internalState, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.setTitle('Live Controller')
		self.liveController = liveController
		self.streamHandler = streamHandler
		self.internal_state = internalState
		self.imaging_channels = CAMERAS.keys()


		self.objective = DEFAULT_OBJECTIVE
		
		self.add_components()
		self.update_pixel_size()

		# self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		self.btn_live = QPushButton("Live")
		self.btn_live.setCheckable(True)
		self.btn_live.setChecked(False)
		self.btn_live.setDefault(False)

		self.checkbox = {}
		for channel in self.imaging_channels:
			self.checkbox[channel] = QCheckBox(channel)

		# 0,1 : choose tracking objective
		self.dropdown_objectiveSelection = QComboBox()
		self.dropdown_objectiveSelection.addItems(list(OBJECTIVES.keys()))
		self.dropdown_objectiveSelection.setCurrentText(DEFAULT_OBJECTIVE)

		# Display resolution slider
		self.slider_resolutionScaling = QSlider(Qt.Horizontal)
		self.slider_resolutionScaling.setTickPosition(QSlider.TicksBelow)
		self.slider_resolutionScaling.setMinimum(10)
		self.slider_resolutionScaling.setMaximum(100)
		self.slider_resolutionScaling.setValue(50)
		self.slider_resolutionScaling.setSingleStep(10)

		self.display_workingResolution = QLCDNumber()
		self.display_workingResolution.setNumDigits(6)
		self.display_workingResolution.display(0.0)

		# @@@ To implement: Multi-image channel support.
		# Each image channel can be checked ON or OFF so that we can switch between multiple imaging modalities
		# This will be a checkbox, where the tracking stream is checked by default.



		# connections

		self.slider_resolutionScaling.valueChanged.connect(self.streamHandler.set_working_resolution_scaling)
		self.slider_resolutionScaling.valueChanged.connect(self.update_image_properties_tracking)
		# self.dropdown_modeSelection.currentIndexChanged.connect(self.update_microscope_mode)
		self.dropdown_objectiveSelection.currentIndexChanged.connect(self.update_pixel_size)
		self.btn_live.clicked.connect(self.toggle_live)

		for channel in self.imaging_channels:
			self.checkbox[channel].clicked.connect(self.update_active_channels)


		# Layout

		# checkbox layout
		checkbox_group = QGroupBox('Imaging channels')
		checkbox_layout = QGridLayout()
		for column, channel in enumerate(self.imaging_channels):
			checkbox_layout.addWidget(self.checkbox[channel],0,column)
		# checkbox_group.setLayout(checkbox_layout)

		objective_layout = QHBoxLayout()
		objective_layout.addWidget(QLabel('Objective'))
		objective_layout.addWidget(self.dropdown_objectiveSelection)
	  
		working_resolution_group = QGroupBox('Working resolution')
		working_resolution_layout = QHBoxLayout()
		working_resolution_layout.addWidget(self.slider_resolutionScaling)
		working_resolution_layout.addWidget(self.display_workingResolution)
		working_resolution_group.setLayout(working_resolution_layout)

		self.grid = QGridLayout()
		self.grid.addWidget(self.btn_live,0,0)
		self.grid.addLayout(objective_layout,0,1)
		self.grid.addLayout(checkbox_layout,1,0)
		self.grid.addWidget(working_resolution_group,1,1)

		self.setLayout(self.grid)

	
	# Slot connected to signal from trackingController.
	def update_working_resolution(self, value):

		self.display_workingResolution.display(value)

	def update_pixel_size(self):
		self.objective = self.dropdown_objectiveSelection.currentText()
		self.internal_state.data['Objective'] = self.objective
		print('Updated internal state objective {}'.format(self.internal_state.data['Objective'] ))
		self.objective_signal.emit(self.objective)
		
	def toggle_live(self,pressed):
		if pressed:
			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)
				if(self.checkbox[channel].isChecked()):
					if(type(self.liveController) is dict):
						self.liveController[channel].start_live()
					else:
						self.liveController.start_live()

					
		else:
			for channel in self.imaging_channels:
				if(type(self.liveController) is dict):
					self.liveController[channel].stop_live()
				else:
					self.liveController.stop_live()
				self.checkbox[channel].setEnabled(True)

	def update_image_properties_tracking(self):

		self.resolution_scaling_signal.emit(self.slider_resolutionScaling.value())

	def update_active_channels(self):

		# @@@ TO DO: Convert these to slots and remove dependency on low level objects
		print('Updating active channels')
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				# Make window active/Show
				self.show_window.emit(True)
			elif (self.checkbox[channel].isChecked()==False):
				# Hide the window.
				self.show_window.emit(False)



class StreamControlWidget(QFrame):
	'''
	Widget controls image/video-stream parameters:
		- Trigger mode (Hardware, Software, Continuous acquisition)
		- Trigger FPS (Set and Actual). Set value only matters during Software trigger. 
		- Display fps (Set and Actual).
	'''

	def __init__(self, streamHandler, liveController, camera, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.liveController = liveController
		self.streamHandler = streamHandler
		self.camera = camera

		self.fps_display = FPS['display']['default']
		self.fps_trigger = FPS['trigger_software']['default']

		self.streamHandler.set_display_fps(self.fps_display)
		self.liveController.set_trigger_fps(self.fps_trigger)
		
		self.add_components()

	def add_components(self):

		# Widgets

		# Trigger Mode
		self.triggerMode = None
		self.dropdown_triggerMode = QComboBox()
		self.dropdown_triggerMode.addItems([TriggerMode.SOFTWARE,TriggerMode.HARDWARE,TriggerMode.CONTINUOUS])
		self.dropdown_triggerMode.setCurrentText(TriggerMode.SOFTWARE)

		# Trigger FPS
		self.entry_triggerFPS = QDoubleSpinBox()
		self.entry_triggerFPS.setMinimum(FPS['trigger_software']['min']) 
		self.entry_triggerFPS.setMaximum(FPS['trigger_software']['max']) 
		self.entry_triggerFPS.setSingleStep(1)
		self.entry_triggerFPS.setValue(self.fps_trigger)

		# Trigger FPS actual
		self.actual_streamFPS = QLCDNumber()
		self.actual_streamFPS.setNumDigits(4)
		self.actual_streamFPS.display(0.0)

		# Entry display fps
		self.entry_displayFPS = QDoubleSpinBox()
		self.entry_displayFPS.setMinimum(FPS['display']['min']) 
		self.entry_displayFPS.setMaximum(FPS['display']['max']) 
		self.entry_displayFPS.setSingleStep(1)
		self.entry_displayFPS.setValue(FPS['display']['default'])

		 # Display fps actual
		self.actual_displayFPS = QLCDNumber()
		self.actual_displayFPS.setNumDigits(4)
		self.actual_displayFPS.display(0.0)


		# Layout
		triggerMode_layout = QHBoxLayout()
		triggerMode_layout.addWidget(QLabel('Trigger mode'))
		triggerMode_layout.addWidget(self.dropdown_triggerMode)


		trigger_fps_group = QGroupBox('Trigger FPS')
		trigger_fps_layout = QGridLayout()
		
		trigger_fps_layout.addWidget(QLabel('Set'),0,0)
		trigger_fps_layout.addWidget(self.entry_triggerFPS, 0,1)
		trigger_fps_layout.addWidget(QLabel('Actual'),0,2)
		trigger_fps_layout.addWidget(self.actual_streamFPS, 0,3)
		trigger_fps_group.setLayout(trigger_fps_layout)

		display_fps_group = QGroupBox('Display FPS')
		display_fps_layout = QGridLayout()
		
		display_fps_layout.addWidget(QLabel('Set'),0,0)
		display_fps_layout.addWidget(self.entry_displayFPS, 0,1)
		display_fps_layout.addWidget(QLabel('Actual'),0,2)
		display_fps_layout.addWidget(self.actual_displayFPS, 0,3)
		display_fps_group.setLayout(display_fps_layout)

		# Overall layout
		self.grid = QGridLayout()
		# self.grid.addLayout(microscope_mode_layout,0,0)
		self.grid.addLayout(triggerMode_layout,0,0)
		self.grid.addWidget(trigger_fps_group,1,0)
		self.grid.addWidget(display_fps_group,2,0)
		self.setLayout(self.grid)

		# Connections
		self.dropdown_triggerMode.currentIndexChanged.connect(self.update_trigger_mode)
		self.entry_displayFPS.valueChanged.connect(self.streamHandler.set_display_fps)
		self.entry_triggerFPS.valueChanged.connect(self.liveController.set_trigger_fps)

	 

	 # Slot connected to signal from streamHandler.
	def update_display_fps(self, value):
		self.actual_displayFPS.display(value)


	# Slot connected to signal from streamHandler.
	def update_stream_fps(self, value):
		self.actual_streamFPS.display(value)

	def update_trigger_mode(self):
		self.liveController.set_trigger_mode(self.dropdown_triggerMode.currentText())

	  

class RecordingWidget(QFrame):
	def __init__(self, streamHandler, imageSaver, internal_state, trackingControllerWidget, trackingDataSaver = None, imaging_channels = TRACKING, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		# In general imageSaver, streamHandler are dicts corresponding to each image_channel
		self.imageSaver = imageSaver # for saving path control
		self.streamHandler = streamHandler
		self.internal_state = internal_state 
		self.trackingDataSaver = trackingDataSaver
		self.trackingControllerWidget = trackingControllerWidget
		self.imaging_channels = imaging_channels

		self.tracking_flag = False
		self.recordingOnly_flag = False

		self.save_dir_base = DEFAULT_SAVE_FOLDER
		self.base_path_is_set = False
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.btn_setSavingDir = QPushButton('Browse')
		self.btn_setSavingDir.setDefault(False)
		self.btn_setSavingDir.setIcon(QIcon('icon/folder.png'))
		
		self.lineEdit_savingDir = QLineEdit()
		self.lineEdit_savingDir.setReadOnly(True)
		self.lineEdit_savingDir.setText('Choose a base saving directory')

		self.lineEdit_experimentID = QLineEdit()

		self.checkbox = {}
		self.entry_saveFPS = {}
		self.actual_saveFPS = {}
		self.entry_timeLimit = {}

		# Check-boxes to select the image channels to save
		for channel in self.imaging_channels:

			self.checkbox[channel] = QCheckBox(channel)
			self.checkbox[channel].setChecked(True)

			# SpinBox for specifying save FPS of each stream

			self.entry_saveFPS[channel] = QDoubleSpinBox()
			self.entry_saveFPS[channel].setMinimum(0.02) 
			self.entry_saveFPS[channel].setMaximum(200) 
			self.entry_saveFPS[channel].setSingleStep(1)
			self.entry_saveFPS[channel].setValue(1)
			self.streamHandler[channel].set_save_fps(1)

			# LCD for displaying the actual save FPS
			self.actual_saveFPS[channel] = QLCDNumber()
			self.actual_saveFPS[channel].setNumDigits(4)
			self.actual_saveFPS[channel].display(0.0)

			# SpinBox for specifying recording time limit for each stream
			self.entry_timeLimit[channel] = QSpinBox()
			self.entry_timeLimit[channel].setMinimum(-1) 
			self.entry_timeLimit[channel].setMaximum(60*60*24*30) 
			self.entry_timeLimit[channel].setSingleStep(1)
			self.entry_timeLimit[channel].setValue(-1)

		

		self.radioButton_tracking = QRadioButton("Track+Record")
		self.radioButton_tracking.setChecked(True)
		self.radioButton_recording = QRadioButton("Record")

		self.btn_record = QPushButton("Record")
		self.btn_record.setCheckable(True)
		self.btn_record.setChecked(False)
		self.btn_record.setDefault(False)

		grid_line1 = QGridLayout()
		grid_line1.addWidget(QLabel('Saving Path'))
		grid_line1.addWidget(self.lineEdit_savingDir, 0,1)
		grid_line1.addWidget(self.btn_setSavingDir, 0,2)

		grid_line2 = QGridLayout()
		grid_line2.addWidget(QLabel('Experiment ID'), 0,0)
		grid_line2.addWidget(self.lineEdit_experimentID,0,1)


		self.tracking_recording_group = QGroupBox()

		tracking_recording_layout = QHBoxLayout()
		tracking_recording_layout.addWidget(self.btn_record)
		tracking_recording_layout.addWidget(self.radioButton_tracking)
		tracking_recording_layout.addWidget(self.radioButton_recording)

		self.tracking_recording_group.setLayout(tracking_recording_layout)


		
		imaging_channel_box = QGroupBox('Imaging channels')

		box_layout = QGridLayout()

		box_layout.addWidget(QLabel('Channel'), 0,0,1,1)
		box_layout.addWidget(QLabel('Save FPS'), 0,1,1,1)
		box_layout.addWidget(QLabel('Actual FPS'), 0,2,1,1)
		box_layout.addWidget(QLabel('Time limit'), 0,3,1,1)

		for row, channel in enumerate(self.imaging_channels):
			box_layout.addWidget(self.checkbox[channel], row+1, 0, 1, 1)
			box_layout.addWidget(self.entry_saveFPS[channel], row+1, 1, 1, 1)
			box_layout.addWidget(self.actual_saveFPS[channel], row+1, 2, 1, 1)
			box_layout.addWidget(self.entry_timeLimit[channel], row+1, 3, 1, 1)


		self.grid = QGridLayout()
		
		self.grid.addLayout(box_layout,0,0,1,1)
	   
		self.grid.addLayout(grid_line1,1,0,1,1)
		self.grid.addLayout(grid_line2,2,0,1,1)

		# self.grid.addWidget(self.btn_record,3,0,1,1)
		self.grid.addWidget(self.tracking_recording_group,3,0,1,1)
		

		self.setLayout(self.grid)

		# add and display a timer - to be implemented
		# self.timer = QTimer()

		# connections
		self.btn_setSavingDir.clicked.connect(self.set_saving_dir)
		self.btn_record.clicked.connect(self.toggle_recording)

		self.radioButton_recording.clicked.connect(self.set_tracking_recording_flag)
		self.radioButton_tracking.clicked.connect(self.set_tracking_recording_flag)



		for channel in self.imaging_channels:
			self.entry_saveFPS[channel].valueChanged.connect(self.streamHandler[channel].set_save_fps)
			self.entry_timeLimit[channel].valueChanged.connect(self.imageSaver[channel].set_recording_time_limit)
			self.imageSaver[channel].stop_recording.connect(self.stop_recording)

	def set_saving_dir(self, use_default_dir = False):
		
		if(use_default_dir is False):
			dialog = QFileDialog()
			self.save_dir_base = dialog.getExistingDirectory(None, "Select Folder")

		# Set base path for image saver
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				self.imageSaver[channel].set_base_path(self.save_dir_base)
		
		# set the base path for the data saver
		if(self.trackingDataSaver is not None):
			self.trackingDataSaver.set_base_path(self.save_dir_base)
		
		self.lineEdit_savingDir.setText(self.save_dir_base)
		self.base_path_is_set = True

	def toggle_recording(self,pressed):
		if self.base_path_is_set == False and self.save_dir_base is None:
			self.btn_record.setChecked(False)
			msg = QMessageBox()
			msg.setText("Please choose base saving directory first")
			msg.exec_()
			return
		elif self.save_dir_base is not None:
			self.set_saving_dir(use_default_dir = True)

		if pressed:
			self.internal_state.data['Acquisition'] = True
			self.lineEdit_experimentID.setEnabled(False)
			self.btn_setSavingDir.setEnabled(False)

			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)

			
			if(self.trackingDataSaver is not None and self.recordingOnly_flag==False):
				self.trackingControllerWidget.btn_track.setChecked(True)
				self.trackingDataSaver.start_new_experiment(self.lineEdit_experimentID.text())
			else:
				pass

			for channel in self.imaging_channels:

				if(self.checkbox[channel].isChecked()):
					self.imageSaver[channel].start_saving_images()
					self.streamHandler[channel].start_recording()

		else:
			self.internal_state.data['Acquisition']= False
			
			for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
			
			self.lineEdit_experimentID.setEnabled(True)
			self.btn_setSavingDir.setEnabled(True)
			

	# stop_recording can be called by imageSaver
	def stop_recording(self):
		self.lineEdit_experimentID.setEnabled(True)
		self.btn_record.setChecked(False)
		for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
		self.btn_setSavingDir.setEnabled(True)

	def set_tracking_recording_flag(self):

		if(self.radioButton_tracking.isChecked()):
			self.recordingOnly_flag = False
			print('Set mode to Tracking+Rec')
		elif(self.radioButton_recording.isChecked()):
			self.recordingOnly_flag = True
			print('Set mode to Recording only')

	def update_save_fps(self, channel, real_fps):

		self.actual_saveFPS[channel].display(real_fps)



'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Plot widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class dockAreaPlot(dock.DockArea):
	def __init__(self, internal_state, parent=None):
		super().__init__(parent)
		self.internal_state = internal_state
		DockLabel.updateStyle = dstyle.updateStylePatched

		self.plots = {key:PlotWidget(key, self.internal_state) for key in PLOT_VARIABLES.keys()}
		
		self.docks = {key:dock.Dock(key) for key in PLOT_VARIABLES.keys()}

		for key in PLOT_VARIABLES.keys():

			self.docks[key].addWidget(self.plots[key])
		
		# Layout of the plots
		
		self.addDock(self.docks['X'])
		self.addDock(self.docks['Z'],'right',self.docks['X'])

		prev_key = 'Z'
		for key in PLOT_VARIABLES:
			if key not in DEFAULT_PLOTS:

				self.addDock(self.docks[key],'above',self.docks[prev_key])
				prev_key = key

	def initialise_plot_area(self):

		for key in self.plots.keys():
			self.plot[key].initialise_plot()





class PlotWidget(pg.GraphicsLayoutWidget):
	def __init__(self,title, internal_state, parent=None):
		super().__init__(parent)
		self.title=title
		self.key = PLOT_VARIABLES[self.title]
		self.internal_state = internal_state
		#plot Zobj
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		
		self.Abs=[]
		self.Ord=[]
		self.plot1=self.addPlot(title=title)
		self.curve=self.plot1.plot(self.Abs,self.Ord)

		self.plot1.enableAutoRange('xy', True)
		self.plot1.showGrid(x=True, y=True)
		
		
	def update_plot(self):

		data = np.zeros(2)
		# For now the x-axis is always time
		data[0] = self.internal_state.data['Time']
		data[1] = self.internal_state.data[self.key]
		
		self.Abscissa.append(data[0])

		self.Ordinate.append(data[1])

		self.label = PLOT_UNITS[self.title]
			
		self.Abs=list(self.Abscissa)
		self.Ord=list(self.Ordinate)

		self.curve.setData(self.Abs,self.Ord)

	def initialise_plot(self):
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		self.Abs=[]
		self.Ord=[]
		self.curve.setData(self.Abs,self.Ord)

# class NavigationWidget(QFrame):
#     def __init__(self, navigationController, main=None, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.navigationController = navigationController
#         self.add_components()
#         self.setFrameStyle(QFrame.Panel | QFrame.Raised)

#     def add_components(self):
#         self.label_Xpos = QLabel()
#         self.label_Xpos.setNum(0)
#         self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dX = QDoubleSpinBox()
#         self.entry_dX.setMinimum(0) 
#         self.entry_dX.setMaximum(5) 
#         self.entry_dX.setSingleStep(0.2)
#         self.entry_dX.setValue(0)
#         self.btn_moveX_forward = QPushButton('Forward')
#         self.btn_moveX_forward.setDefault(False)
#         self.btn_moveX_backward = QPushButton('Backward')
#         self.btn_moveX_backward.setDefault(False)
		
#         self.label_Ypos = QLabel()
#         self.label_Ypos.setNum(0)
#         self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dY = QDoubleSpinBox()
#         self.entry_dY.setMinimum(0)
#         self.entry_dY.setMaximum(5)
#         self.entry_dY.setSingleStep(0.2)
#         self.entry_dY.setValue(0)
#         self.btn_moveY_forward = QPushButton('Forward')
#         self.btn_moveY_forward.setDefault(False)
#         self.btn_moveY_backward = QPushButton('Backward')
#         self.btn_moveY_backward.setDefault(False)

#         self.label_Zpos = QLabel()
#         self.label_Zpos.setNum(0)
#         self.label_Zpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
#         self.entry_dZ = QDoubleSpinBox()
#         self.entry_dZ.setMinimum(0) 
#         self.entry_dZ.setMaximum(1000) 
#         self.entry_dZ.setSingleStep(0.2)
#         self.entry_dZ.setValue(0)
#         self.btn_moveZ_forward = QPushButton('Forward')
#         self.btn_moveZ_forward.setDefault(False)
#         self.btn_moveZ_backward = QPushButton('Backward')
#         self.btn_moveZ_backward.setDefault(False)
		
#         grid_line0 = QGridLayout()
#         grid_line0.addWidget(QLabel('X (mm)'), 0,0)
#         grid_line0.addWidget(self.label_Xpos, 0,1)
#         grid_line0.addWidget(self.entry_dX, 0,2)
#         grid_line0.addWidget(self.btn_moveX_forward, 0,3)
#         grid_line0.addWidget(self.btn_moveX_backward, 0,4)

#         grid_line1 = QGridLayout()
#         grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
#         grid_line1.addWidget(self.label_Ypos, 0,1)
#         grid_line1.addWidget(self.entry_dY, 0,2)
#         grid_line1.addWidget(self.btn_moveY_forward, 0,3)
#         grid_line1.addWidget(self.btn_moveY_backward, 0,4)

#         grid_line2 = QGridLayout()
#         grid_line2.addWidget(QLabel('Z (um)'), 0,0)
#         grid_line2.addWidget(self.label_Zpos, 0,1)
#         grid_line2.addWidget(self.entry_dZ, 0,2)
#         grid_line2.addWidget(self.btn_moveZ_forward, 0,3)
#         grid_line2.addWidget(self.btn_moveZ_backward, 0,4)

#         self.grid = QGridLayout()
#         self.grid.addLayout(grid_line0,0,0)
#         self.grid.addLayout(grid_line1,1,0)
#         self.grid.addLayout(grid_line2,2,0)
#         self.setLayout(self.grid)

#         self.btn_moveX_forward.clicked.connect(self.move_x_forward)
#         self.btn_moveX_backward.clicked.connect(self.move_x_backward)
#         self.btn_moveY_forward.clicked.connect(self.move_y_forward)
#         self.btn_moveY_backward.clicked.connect(self.move_y_backward)
#         self.btn_moveZ_forward.clicked.connect(self.move_z_forward)
#         self.btn_moveZ_backward.clicked.connect(self.move_z_backward)
		
#     def move_x_forward(self):
#         self.navigationController.move_x(self.entry_dX.value())
#         print('move x')
#     def move_x_backward(self):
#         self.navigationController.move_x(-self.entry_dX.value())
#     def move_y_forward(self):
#         self.navigationController.move_y(self.entry_dY.value())
#     def move_y_backward(self):
#         self.navigationController.move_y(-self.entry_dY.value())
#     def move_z_forward(self):
#         self.navigationController.move_z(self.entry_dZ.value()/1000)
#     def move_z_backward(self):
#         self.navigationController.move_z(-self.entry_dZ.value()/1000)

class AutoFocusWidget(QFrame):
	def __init__(self, autofocusController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.autofocusController = autofocusController
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.entry_delta = QDoubleSpinBox()
		self.entry_delta.setMinimum(0.2) 
		self.entry_delta.setMaximum(20) 
		self.entry_delta.setSingleStep(0.2)
		self.entry_delta.setValue(3)
		self.autofocusController.set_deltaZ(3)

		self.entry_N = QSpinBox()
		self.entry_N.setMinimum(3) 
		self.entry_N.setMaximum(20) 
		self.entry_N.setSingleStep(1)
		self.entry_N.setValue(10)
		self.autofocusController.set_N(10)

		self.btn_autofocus = QPushButton('Autofocus')
		self.btn_autofocus.setDefault(False)
		self.btn_autofocus.setCheckable(True)
		self.btn_autofocus.setChecked(False)

		# layout
		grid_line0 = QGridLayout()
		grid_line0.addWidget(QLabel('delta Z (um)'), 0,0)
		grid_line0.addWidget(self.entry_delta, 0,1)
		grid_line0.addWidget(QLabel('N Z planes'), 0,2)
		grid_line0.addWidget(self.entry_N, 0,3)
		grid_line0.addWidget(self.btn_autofocus, 0,4)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)
		self.setLayout(self.grid)
		
		# connections
		self.btn_autofocus.clicked.connect(self.autofocusController.autofocus)
		self.entry_delta.valueChanged.connect(self.autofocusController.set_deltaZ)
		self.entry_N.valueChanged.connect(self.autofocusController.set_N)
		self.autofocusController.autofocusFinished.connect(self.autofocus_is_finished)

	def autofocus_is_finished(self):
		self.btn_autofocus.setChecked(False)

class DishScanWidget(QFrame):
    def __init__(self, multipointController, dishscanController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.multipointController = multipointController
        self.dishscanController = dishscanController
        #insert dishscanController as well and change all dependencies from multipoint to dishscan
        self.base_path_is_set = False
        self.home_is_set = True #False  change if need to implement select home button
        self.period_is_set = False
        self.template_is_set = False
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.btn_setSavingDir = QPushButton('Browse')
        self.btn_setSavingDir.setDefault(False)
        self.btn_setSavingDir.setIcon(QIcon('icon/folder.png'))
        
        self.lineEdit_savingDir = QLineEdit()
        self.lineEdit_savingDir.setReadOnly(False)
        self.lineEdit_savingDir.setText('Choose a base saving directory')

        self.lineEdit_experimentID = QLineEdit()
        self.lineEdit_experimentID.setReadOnly(False)
        self.lineEdit_experimentID .setText('Enter ID to distinguish files')

        self.btn_startAcquisition = QPushButton('Start Acquisition')
        self.btn_startAcquisition.setCheckable(True)
        self.btn_startAcquisition.setChecked(False)
        self.btn_startAcquisition.setDefault(False)

        self.btn_scanPeriod = QPushButton('Save Scan Period(s) and Loops')

        # self.btn_setHome = QPushButton('Current Position as Home')   #set home button functionality appropriately create new functions as required

        self.entry_dt = QDoubleSpinBox()
        self.entry_dt.setMinimum(0) 
        self.entry_dt.setMaximum(86400) 
        self.entry_dt.setSingleStep(1)
        self.entry_dt.setValue(900)

        self.entry_Nt = QSpinBox()
        self.entry_Nt.setMinimum(1) 
        self.entry_Nt.setMaximum(50000)   # @@@ to be changed
        self.entry_Nt.setSingleStep(1)
        self.entry_Nt.setValue(1)

        self.checkbox_subsIllumination = QCheckBox('Substrate Illumination')
        self.checkbox_offnotScanning = QCheckBox('Turn off when idle')
        self.checkbox_scanSave = QCheckBox('Scan Save')
        self.checkbox_withAutofocus = QCheckBox('With Autofocus')

        self.comboBox = QComboBox()    #set comboBox functionality appropriately, create new functions for the same
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("9 Pelco 50 mm Dishes")
        self.comboBox.addItem("6 Well Plate")

        self.lineEdit_wellID = QLineEdit()
        self.lineEdit_wellID.setReadOnly(False)
        self.lineEdit_wellID .setText('Enter ID for wells to scan')

        self.btn_chooseTemplate = QPushButton('Choose Template')

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Saving Path'))
        grid_line1.addWidget(self.lineEdit_savingDir, 0,1)
        grid_line1.addWidget(self.btn_setSavingDir, 0,2)

        grid_line3 = QGridLayout()
        grid_line3.addWidget(QLabel('dt (s) (time between scans)'), 0,0)
        grid_line3.addWidget(self.entry_dt, 0,1)
        grid_line3.addWidget(QLabel('Nt ( number of loops)'), 0,2)
        grid_line3.addWidget(self.entry_Nt, 0,3)
        grid_line3.addWidget(self.btn_scanPeriod, 0,4)
        # grid_line3.addWidget(self.btn_setHome,0,2)

        grid_line4 = QGridLayout()
        grid_line4.addWidget(self.comboBox,0,0)
        grid_line4.addWidget(self.lineEdit_wellID,0,1)
        grid_line4.addWidget(self.btn_chooseTemplate,0,2)

        grid_line5 = QGridLayout()
        grid_line5.addWidget(QLabel('Well IDs are counted from top left corner as the origin of the plate. Then follow reading conventions.'),0,0)

        grid_line6 = QGridLayout()
        grid_line6.addWidget(QLabel('Mention IDs of the wells to be imaged as a string separated by spaces. For eg. 1 2 8 9. Stay within limits of the template.'),0,0)

        grid_line7 = QGridLayout()
        grid_line7.addWidget(self.checkbox_subsIllumination, 0,0)
        grid_line7.addWidget(self.checkbox_offnotScanning, 0,1)
        grid_line7.addWidget(self.checkbox_scanSave, 0,2)
        grid_line7.addWidget(self.checkbox_withAutofocus, 0,3)

        grid_line8 = QGridLayout()
        grid_line8.addWidget(QLabel('Scan Save: Scan for animal and only save image along with coordinates if an animal is seen.'),0,0)

        grid_line9 = QGridLayout()
        grid_line9.addWidget(QLabel('The animal is centered in the frame before taking the image. If not selected all images are saved.'),0,0)

        grid_line2 = QGridLayout()
        grid_line2.addWidget(QLabel('Experiment ID'), 0,0)
        grid_line2.addWidget(self.lineEdit_experimentID,0,1)
        grid_line2.addWidget(self.btn_startAcquisition, 0,2)

        self.grid = QGridLayout()
        self.grid.addLayout(grid_line1,0,0)
        self.grid.addLayout(grid_line3,1,0)
        self.grid.addLayout(grid_line4,2,0)
        self.grid.addLayout(grid_line5,3,0)
        self.grid.addLayout(grid_line6,4,0)
        self.grid.addLayout(grid_line7,5,0)
        self.grid.addLayout(grid_line8,6,0)
        self.grid.addLayout(grid_line9,7,0)
        self.grid.addLayout(grid_line2,8,0)  #note the order is mismatched, gridline2 is in the end
        self.setLayout(self.grid)

        # connections
        self.checkbox_subsIllumination.stateChanged.connect(self.dishscanController.set_subsill_flag)      #use dishscanController instead of multipointController
        self.checkbox_offnotScanning.stateChanged.connect(self.dishscanController.set_offnot_flag)
        self.checkbox_withAutofocus.stateChanged.connect(self.dishscanController.set_af_flag)
        self.checkbox_scanSave.stateChanged.connect(self.dishscanController.set_scansave_flag)
        self.btn_setSavingDir.clicked.connect(self.set_saving_dir)
        self.btn_startAcquisition.clicked.connect(self.toggle_acquisition)
        # self.btn_setHome.clicked.connect(self.set_home_coord)
        self.btn_scanPeriod.clicked.connect(self.set_scan_period)
        self.btn_chooseTemplate.clicked.connect(self.set_dish_template)
        #new buttons for saving time period and dish template
        self.dishscanController.acquisitionFinished.connect(self.acquisition_is_finished)  #once the finished signal is emitted, then execute the acquisition finished function

    # not using currently, but use if necessary by actually receiving feedback signal from the motors.
    def set_home_coord(self):
        home_coord =  [0,0]####get coordinates from microcontrollerm. Use if required, otherwise just consider relative to the position where scan starts
        self.dishscanController.set_home(home_coord)
        self.home_is_set = True
        print(self.base_path_is_set, self.home_is_set, self.period_is_set, self.template_is_set)

    def set_scan_period(self):
        scan_period = self.entry_dt.value()
        scan_loops = self.entry_Nt.value()
        self.dishscanController.set_period(scan_period, scan_loops)
        self.period_is_set = True
        print(self.base_path_is_set, self.home_is_set, self.period_is_set, self.template_is_set)
        ####talk to core to tell it to save home coordinates

    def set_dish_template(self):
        dish_template = self.comboBox.currentIndex()
        well_ID_string = self.lineEdit_wellID.text()
        if (well_ID_string == ""):
            msg = QMessageBox()
            msg.setText("Please enter the IDs for wells you wish to image in the written format")
            msg.exec_()
            return
        well_ID_list = list(map(int,well_ID_string.split()))
        #check if well ids are within template and then then only proceed, otherwise show error
        if (dish_template == 0):
            if max(well_ID_list)>9 or min(well_ID_list)<1:
                msg = QMessageBox()
                msg.setText("Please enter well IDs in the range 1-9")
                msg.exec_()
                return
        if (dish_template == 1):
            if max(well_ID_list)>6 or min(well_ID_list)<1:
                msg = QMessageBox()
                msg.setText("Please enter well IDs in the range 1-6")
                msg.exec_()
                return
        self.dishscanController.set_template(dish_template, well_ID_list)
        self.template_is_set = True
        print(self.base_path_is_set, self.home_is_set, self.period_is_set, self.template_is_set)

    def set_saving_dir(self):
        dialog = QFileDialog()
        save_dir_base = dialog.getExistingDirectory(None, "Select Folder")
        self.dishscanController.set_base_path(save_dir_base)
        self.lineEdit_savingDir.setText(save_dir_base)
        self.base_path_is_set = True
        print(self.base_path_is_set, self.home_is_set, self.period_is_set, self.template_is_set)

    def toggle_acquisition(self,pressed):
        if (self.base_path_is_set == False) or (self.home_is_set == False) or (self.period_is_set == False) or (self.template_is_set == False):
            self.btn_startAcquisition.setChecked(False)
            msg = QMessageBox()
            msg.setText("Please choose base saving directory, home position, scan period, and dish template first")
            msg.exec_()
            return
        if self.btn_startAcquisition.isChecked(): #pressed:
            # @@@ to do: add a widgetManger to enable and disable widget 
            # @@@ to do: emit signal to widgetManager to disable other widgets
            self.setEnabled_all(False)
            self.dishscanController.start_new_experiment(self.lineEdit_experimentID.text())
            self.dishscanController.run_acquisition()
        else:
            # self.multipointController.stop_acquisition() # to implement
            self.setEnabled_all(True)
            self.base_path_is_set = False
            # self.home_is_set = False
            self.period_is_set = False
            self.template_is_set = False
            print(self.base_path_is_set, self.home_is_set, self.period_is_set, self.template_is_set)
            msg = QMessageBox()
            msg.setText("Please reset directory, scan period, home, and template")
            msg.exec_()
            return

    def acquisition_is_finished(self):
        self.btn_startAcquisition.setChecked(False)
        self.setEnabled_all(True)
        self.base_path_is_set = False
        # self.home_is_set = False
        self.period_is_set = False
        self.template_is_set = False

    def setEnabled_all(self,enabled,exclude_btn_startAcquisition=True):
        self.btn_setSavingDir.setEnabled(enabled)
        # self.btn_setHome.setEnabled(enabled)
        self.lineEdit_savingDir.setEnabled(enabled)
        self.lineEdit_experimentID.setEnabled(enabled)
        self.lineEdit_wellID.setEnabled(enabled)
        self.checkbox_subsIllumination.setEnabled(enabled)
        self.checkbox_offnotScanning.setEnabled(enabled)
        self.checkbox_scanSave.setEnabled(enabled)
        self.checkbox_withAutofocus.setEnabled(enabled)
        self.comboBox.setEnabled(enabled)
        self.btn_scanPeriod.setEnabled(enabled)
        self.btn_chooseTemplate.setEnabled(enabled)
        self.entry_dt.setEnabled(enabled)
        self.entry_Nt.setEnabled(enabled)
        if exclude_btn_startAcquisition is not True:
            self.btn_startAcquisition.setEnabled(enabled)

class MultiPointWidget(QFrame):
    def __init__(self, multipointController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.multipointController = multipointController
        self.base_path_is_set = False
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):

        self.btn_setSavingDir = QPushButton('Browse')
        self.btn_setSavingDir.setDefault(False)
        self.btn_setSavingDir.setIcon(QIcon('icon/folder.png'))
        
        self.lineEdit_savingDir = QLineEdit()
        self.lineEdit_savingDir.setReadOnly(True)
        self.lineEdit_savingDir.setText('Choose a base saving directory')

        self.lineEdit_experimentID = QLineEdit()

        self.entry_deltaX = QDoubleSpinBox()
        self.entry_deltaX.setMinimum(0.2) 
        self.entry_deltaX.setMaximum(5) 
        self.entry_deltaX.setSingleStep(1)
        self.entry_deltaX.setValue(Acquisition.DX)

        self.entry_NX = QSpinBox()
        self.entry_NX.setMinimum(1) 
        self.entry_NX.setMaximum(20) 
        self.entry_NX.setSingleStep(1)
        self.entry_NX.setValue(1)

        self.entry_deltaY = QDoubleSpinBox()
        self.entry_deltaY.setMinimum(0.2) 
        self.entry_deltaY.setMaximum(5) 
        self.entry_deltaY.setSingleStep(1)
        self.entry_deltaY.setValue(Acquisition.DX)
        
        self.entry_NY = QSpinBox()
        self.entry_NY.setMinimum(1) 
        self.entry_NY.setMaximum(20) 
        self.entry_NY.setSingleStep(1)
        self.entry_NY.setValue(1)

        self.entry_deltaZ = QDoubleSpinBox()
        self.entry_deltaZ.setMinimum(0) 
        self.entry_deltaZ.setMaximum(1000) 
        self.entry_deltaZ.setSingleStep(0.2)
        self.entry_deltaZ.setValue(Acquisition.DZ)
        
        self.entry_NZ = QSpinBox()
        self.entry_NZ.setMinimum(1) 
        self.entry_NZ.setMaximum(100) 
        self.entry_NZ.setSingleStep(1)
        self.entry_NZ.setValue(1)
        

        self.entry_dt = QDoubleSpinBox()
        self.entry_dt.setMinimum(0) 
        self.entry_dt.setMaximum(3600) 
        self.entry_dt.setSingleStep(1)
        self.entry_dt.setValue(1)

        self.entry_Nt = QSpinBox()
        self.entry_Nt.setMinimum(1) 
        self.entry_Nt.setMaximum(50000)   # @@@ to be changed
        self.entry_Nt.setSingleStep(1)
        self.entry_Nt.setValue(1)

        self.checkbox_bfdf = QCheckBox('BF/DF')
        self.checkbox_fluorescence = QCheckBox('Fluorescence')
        self.checkbox_withAutofocus = QCheckBox('With AF')
        self.btn_startAcquisition = QPushButton('Start Acquisition')
        self.btn_startAcquisition.setCheckable(True)
        self.btn_startAcquisition.setChecked(False)

        # layout
        grid_line0 = QGridLayout()
        grid_line0.addWidget(QLabel('Saving Path'))
        grid_line0.addWidget(self.lineEdit_savingDir, 0,1)
        grid_line0.addWidget(self.btn_setSavingDir, 0,2)

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Experiment ID'), 0,0)
        grid_line1.addWidget(self.lineEdit_experimentID,0,1)

        grid_line2 = QGridLayout()
        grid_line2.addWidget(QLabel('dx (mm)'), 0,0)
        grid_line2.addWidget(self.entry_deltaX, 0,1)
        grid_line2.addWidget(QLabel('Nx'), 0,2)
        grid_line2.addWidget(self.entry_NX, 0,3)
        grid_line2.addWidget(QLabel('dy (mm)'), 0,4)
        grid_line2.addWidget(self.entry_deltaY, 0,5)
        grid_line2.addWidget(QLabel('Ny'), 0,6)
        grid_line2.addWidget(self.entry_NY, 0,7)

        grid_line2.addWidget(QLabel('dz (um)'), 1,0)
        grid_line2.addWidget(self.entry_deltaZ, 1,1)
        grid_line2.addWidget(QLabel('Nz'), 1,2)
        grid_line2.addWidget(self.entry_NZ, 1,3)
        grid_line2.addWidget(QLabel('dt (s)'), 1,4)
        grid_line2.addWidget(self.entry_dt, 1,5)
        grid_line2.addWidget(QLabel('Nt'), 1,6)
        grid_line2.addWidget(self.entry_Nt, 1,7)

        grid_line3 = QHBoxLayout()
        grid_line3.addWidget(self.checkbox_bfdf)
        grid_line3.addWidget(self.checkbox_fluorescence)
        grid_line3.addWidget(self.checkbox_withAutofocus)
        grid_line3.addWidget(self.btn_startAcquisition)

        self.grid = QGridLayout()
        self.grid.addLayout(grid_line0,0,0)
        self.grid.addLayout(grid_line1,1,0)
        self.grid.addLayout(grid_line2,2,0)
        self.grid.addLayout(grid_line3,3,0)
        self.setLayout(self.grid)

        # add and display a timer - to be implemented
        # self.timer = QTimer()

        # connections
        self.entry_deltaX.valueChanged.connect(self.multipointController.set_deltaX)
        self.entry_deltaY.valueChanged.connect(self.multipointController.set_deltaY)
        self.entry_deltaZ.valueChanged.connect(self.multipointController.set_deltaZ)
        self.entry_dt.valueChanged.connect(self.multipointController.set_deltat)
        self.entry_NX.valueChanged.connect(self.multipointController.set_NX)
        self.entry_NY.valueChanged.connect(self.multipointController.set_NY)
        self.entry_NZ.valueChanged.connect(self.multipointController.set_NZ)
        self.entry_Nt.valueChanged.connect(self.multipointController.set_Nt)
        self.checkbox_bfdf.stateChanged.connect(self.multipointController.set_bfdf_flag)
        self.checkbox_fluorescence.stateChanged.connect(self.multipointController.set_fluorescence_flag)
        self.checkbox_withAutofocus.stateChanged.connect(self.multipointController.set_af_flag)
        self.btn_setSavingDir.clicked.connect(self.set_saving_dir)
        self.btn_startAcquisition.clicked.connect(self.toggle_acquisition)
        self.multipointController.acquisitionFinished.connect(self.acquisition_is_finished)

    def set_saving_dir(self):
        dialog = QFileDialog()
        save_dir_base = dialog.getExistingDirectory(None, "Select Folder")
        self.multipointController.set_base_path(save_dir_base)
        self.lineEdit_savingDir.setText(save_dir_base)
        self.base_path_is_set = True

    def toggle_acquisition(self,pressed):
        if self.base_path_is_set == False:
            self.btn_startAcquisition.setChecked(False)
            msg = QMessageBox()
            msg.setText("Please choose base saving directory first")
            msg.exec_()
            return
        if pressed:
            # @@@ to do: add a widgetManger to enable and disable widget 
            # @@@ to do: emit signal to widgetManager to disable other widgets
            self.setEnabled_all(False)
            self.multipointController.start_new_experiment(self.lineEdit_experimentID.text())
            self.multipointController.run_acquisition()
        else:
            # self.multipointController.stop_acquisition() # to implement
            self.setEnabled_all(True)

    def acquisition_is_finished(self):
        self.btn_startAcquisition.setChecked(False)
        self.setEnabled_all(True)

    def setEnabled_all(self,enabled,exclude_btn_startAcquisition=True):
        self.btn_setSavingDir.setEnabled(enabled)
        self.lineEdit_savingDir.setEnabled(enabled)
        self.lineEdit_experimentID.setEnabled(enabled)
        self.entry_deltaX.setEnabled(enabled)
        self.entry_NX.setEnabled(enabled)
        self.entry_deltaY.setEnabled(enabled)
        self.entry_NY.setEnabled(enabled)
        self.entry_deltaZ.setEnabled(enabled)
        self.entry_NZ.setEnabled(enabled)
        self.entry_dt.setEnabled(enabled)
        self.entry_Nt.setEnabled(enabled)
        self.checkbox_bfdf.setEnabled(enabled)
        self.checkbox_fluorescence.setEnabled(enabled)
        self.checkbox_withAutofocus.setEnabled(enabled)
        if exclude_btn_startAcquisition is not True:
            self.btn_startAcquisition.setEnabled(enabled)
