class TriggerMode:
    SOFTWARE = 'Software Trigger'
    HARDWARE = 'Hardware Trigger'
    CONTINUOUS = 'Continuous Acqusition'
    def __init__(self):
        pass

class MicroscopeMode:
    BFDF = 'BF/DF'
    FLUORESCENCE = 'Fluorescence'
    FLUORESCENCE_PREVIEW = 'Fluorescence Preview'
    def __init__(self):
        pass

class WaitTime:
    BASE = 0.1
    X = 0.4     # per mm
    Y = 0.4	 # per mm
    Z = 0.2     # per mm
    def __init__(self):
        pass

class AF:
    STOP_THRESHOLD = 0.85
    CROP_WIDTH = 500
    CROP_HEIGHT = 500
    def __init__(self):
        pass

class Motion:
    STEPS_PER_MM_XY = 50 # microsteps
    STEPS_PER_MM_Z = 1600  # microsteps
    def __init__(self):
        pass
'''
# for octopi-malaria
class Motion:
    STEPS_PER_MM_XY = 40
    STEPS_PER_MM_Z = 5333
    def __init__(self):
        pass
'''

class Acquisition:
    CROP_WIDTH = 1930  #for getting edge width of 4mm at 5x magnification and 10mm at 2x magnification
    CROP_HEIGHT = 1930
    NUMBER_OF_FOVS_PER_AF = 3
    IMAGE_FORMAT = 'tif'
    IMAGE_DISPLAY_SCALING_FACTOR = 0.25
    DX = 1 #in mm 
    DY = 1 #in mm, #####make sure that these are correctly implemented in the hardware
    DZ = 3    #what are the units for these?

    def __init__(self):
        pass

class DishAcquisition:
    CROP_WIDTH = 1930  #for getting edge width of 4mm at 5x magnification and 10mm at 2x magnification
    CROP_HEIGHT = 1930
    NUMBER_OF_FOVS_PER_AF = 3
    IMAGE_FORMAT = 'tif'
    IMAGE_DISPLAY_SCALING_FACTOR = 0.25
    OVERLAP = 0.5  #fraction of frame to shift while scanning in one direction. 1/3 would enforce a shift of 1/3rd of each frame width.
    WIDTHX = 40 #in mm, 0.5*40/5 = 4mm for 5x and 0.5*40/2 = 10mm for 2x
    WIDTHY = 40 #in mm, 10mm for 2x #####make sure that these are correctly implemented in the hardware
    DZ = 3 #in mm here, later converted into microns in the core code   #what are the units for these?

    def __init__(self):
        pass

class PosUpdate:
    INTERVAL_MS = 25

class MicrocontrollerDef:
    MSG_LENGTH = 9
    CMD_LENGTH = 4
    N_BYTES_POS = 3

class TemplateDef:
    scale_5x_pxmm = 479
    scale_2x_pxmm = 191.6
    pelco_50_sz_mm = 50
    pelco_50_sep_mm = 15
