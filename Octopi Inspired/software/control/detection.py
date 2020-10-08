#code to detect an animal in a frame

import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib.pylab as pl

import numpy as np

import os
from skimage import io, color, img_as_ubyte, filters, util, exposure, measure, morphology

from skimage.feature import canny

from skimage.morphology import disk

from skimage.segmentation import clear_border


def detect_animal(image):
    is_animal = False
    scale = 2.073  # in microns/pixel  as all the datasets are 5x
    disk_sz = 10  # disk size for median filter, need enough smoothening but not too large to make it unable to detect features
    thresh_val = 200 
    
    width, height = image.shape
    trial8 = img_as_ubyte(image)   #[width//2-965:width//2+965, height//2-965:height//2+965])
    equ_trial8 = (255*exposure.equalize_adapthist(trial8)).astype(int)
    
    selem = disk(disk_sz)
    closed_trial8 = morphology.closing(equ_trial8, selem)
    
    closed_edge = canny(closed_trial8, sigma=5, low_threshold=0, high_threshold=30)   #adjust these depending on the image, use gravity machine controller to make a drag based controller for these values
    
    edge_coord = np.where(closed_edge==1)
    
#    x_coord = edge_coord[0] - np.mean(edge_coord[0])
#    y_coord = edge_coord[1] - np.mean(edge_coord[1])
    cent_coord = [np.mean(edge_coord[0]), np.mean(edge_coord[1])]
    
  #  cov = np.cov(x_coord, y_coord)
 #   w, v = np.linalg.eig(cov)
    
#    n_std = 1.414

#    width = n_std*2*np.sqrt(w[0])
#    height = n_std*2*np.sqrt(w[1])
#    angle = np.arctan(v[1,0]/v[0,0])*360/(2*np.pi)
    
    edge_peri = measure.perimeter(closed_edge, neighbourhood=2)
    
    if edge_peri > 100:
        is_animal = True
        
    return is_animal, cent_coord


def check_saved(cent_coord):
    
    return
