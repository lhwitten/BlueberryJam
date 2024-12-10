import cv2 as cv
import numpy as np

default = (0.7268992857209939,0.9510403996705553,0.9259793130907451)

def scale_matlike(channel, frac:float):
    return np.clip(channel*frac,0,255)

def adjust_colorspace(lab_img, frac:tuple = default):
    '''
    Adjusts an image to loosely align with the belt prototype lighting conditions

    Params:

    - lab_img: lab image to be adjusted

    - frac: fraction set with which to scale the color channels of the image

    Returns:
    an adjusted BGR image
    '''
    l, a, b = cv.split(lab_img)
 
    l = scale_matlike(l, frac[0])
    a = scale_matlike(a, frac[1])
    b = scale_matlike(b, frac[2])

    l = cv.convertScaleAbs(l)
    a = cv.convertScaleAbs(a)
    b = cv.convertScaleAbs(b)
    img = cv.merge((l,a,b))
    return cv.cvtColor(img, cv.COLOR_LAB2BGR)
    
