import cv2 as cv
import numpy as np

#TODO: create ground truth vector for comparison to ripe berries

def crop_single_luv_img(img_rgb, centroid:tuple):
# get centroid'd image in RGB
# convert to LUV
    img_luv = cv.cvtColor(img_rgb, cv.COLOR_RGB2LUV)

# retrieve box dimensions
    [x,y,w,h] = centroid[2]

# crop img into smaller images using box dims
    return img_luv[y:y+h, x:x+w]


def norm(vector:list):
    return np.sqrt(sum(x * x for x in vector))    


def cosine_similarity(vec_a:list, vec_b:list) -> float:
        norm_a = norm(vec_a)
        norm_b = norm(vec_b)
        dot = sum(a * b for a, b in zip(vec_a, vec_b))
        return dot / (norm_a * norm_b) if norm_a * norm_b != 0 else 0.0


def create_def_vectors(img_luv) -> tuple:
# split color channels into L, U, and V
    l_channel, u_channel, v_channel = cv.split(img_luv)

    histSize = 256
    histRange = (0, 255)

# find hist vector from L channel
    lcounts = np.histogram(l_channel.flatten(), bins=histSize, range=histRange)
    lvalues = lcounts[0].tolist()

# find hist vector from U channel
    ucounts = np.histogram(u_channel.flatten(), bins=histSize, range=histRange)
    uvalues = ucounts[0].tolist()

# find hist vector from V channel
    vcounts = np.histogram(v_channel.flatten(), bins=histSize, range=histRange)
    vvalues = vcounts[0].tolist()

    return (lvalues,uvalues,vvalues)

def compare_img_vectors(img_a:tuple, img_b:tuple) -> float:
# calculate cosine simularity for all channels
    lsim = cosine_similarity(img_a[0], img_b[0])
    usim = cosine_similarity(img_a[1], img_b[1])
    vsim = cosine_similarity(img_a[2], img_b[2])

# return the average of the similarities
    return np.average([lsim, usim, vsim])
