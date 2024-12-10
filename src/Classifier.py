import os
from datetime import datetime
import cv2 as cv
import numpy as np
# from ripe_avg_val import ripe_avg_val
# from overripe_avg_val import overripe_avg_val
# from unripe_avg_val import unripe_avg_val
#from numpy import flatten
SIM_THRESH = 0.73

#TODO: create ground truth vector for comparison to ripe berries
def crop_single_luv_img(img_rgb, centroid:tuple):
# get centroid'd image in RGB
# convert to LUV
    img_luv = cv.cvtColor(img_rgb, cv.COLOR_RGB2LUV)
# retrieve box dimensions
    [x,y,w,h] = centroid[2]
# crop img into smaller images using box dims
    height, width = img_luv.shape[:2]
    if x < 0 or y < 0 or x + w > width or y + h > height:
        return []
    return img_luv[y:y+h, x:x+w]
# def norm(vector:list):
#     return np.sqrt(sum(x * x for x in vector))    
# def cosine_similarity(vec_a:list, vec_b:list) -> float:
#         norm_a = norm(vec_a)
#         norm_b = norm(vec_b)
#         dot = sum(a * b for a, b in zip(vec_a, vec_b))
#         return dot / (norm_a * norm_b) if norm_a * norm_b != 0 else 0.0
# def create_def_vectors(img_luv) -> tuple:
# # split color channels into L, U, and V
#     # print("got here 0")
#     l_channel, u_channel, v_channel = cv.split(img_luv)
#     histSize = 256
#     histRange = (0, 255)
#     # print("got here 1")
# # find hist vector from L channel
#     lcounts = np.histogram(l_channel.flatten(), bins=histSize, range=histRange)
#     lvalues = lcounts[0].tolist()
#     # print("got here 2")
# # find hist vector from U channel
#     ucounts = np.histogram(u_channel.flatten(), bins=histSize, range=histRange)
#     uvalues = ucounts[0].tolist()
# # find hist vector from V channel
#     vcounts = np.histogram(v_channel.flatten(), bins=histSize, range=histRange)
#     vvalues = vcounts[0].tolist()
#     return (lvalues,uvalues,vvalues)
# def compare_img_vectors(img_a:tuple, img_b:tuple) -> float:
# # calculate cosine simularity for all channels
#     lsim = cosine_similarity(img_a[0], img_b[0])
#     usim = cosine_similarity(img_a[1], img_b[1])
#     vsim = cosine_similarity(img_a[2], img_b[2])
# # return the average of the similarities
#     return np.average([lsim, usim, vsim])

# def compare_to_avgs(img: tuple) -> float:
#     ripe_sim = compare_img_vectors(img, ripe_avg_val)
#     overripe_sim = compare_img_vectors(img, overripe_avg_val)
#     unripe_sim = compare_img_vectors(img, unripe_avg_val)
#     return (ripe_sim, overripe_sim, unripe_sim)

def check_ripeness(ripeness_sims: tuple) -> int:
    return ripeness_sims.index(max(ripeness_sims))

def classify_single(img_rgb,img_copy, single_centroid):
   img_luv  = crop_single_luv_img(img_rgb, single_centroid)
   #print("got here -1")
   if img_luv == []:
    #print("img_luv is None")
    return -5
   #print(img_luv)

#    def_vectors = create_def_vectors(img_luv)

#    similarity_vec = compare_to_avgs(def_vectors)

#    is_ripe = check_ripeness(similarity_vec)

#    annotate_and_show(img_copy,single_centroid,similarity_vec,is_ripe)
#    is_ripe = 1
#    return is_ripe

def annotate_and_show(img_copy,centroid,similarity,ripeness):
    #[x_min,y_min,width,height]
    if ripeness == 0:
        text = f"ripe:{round(similarity[0],3)}"
        subtext = f"R:{round(similarity[0],3)}"
        sub2 =   f"O: {round(similarity[1],3)}"
        sub3 =   f"U: {round(similarity[2],3)}"
    elif ripeness == 1:
        text = f"overripe:{round(similarity[1],3)}"
        subtext = f"R:{round(similarity[0],3)}"
        sub2 =   f"O: {round(similarity[1],3)}"
        sub3 =   f"U: {round(similarity[2],3)}"
    else:
        text = f"unripe:{round(similarity[2],3)}"
        subtext = f"R:{round(similarity[0],3)}"
        sub2 =   f"O: {round(similarity[1],3)}"
        sub3 =   f"U: {round(similarity[2],3)}"
    
    position = (centroid[0],centroid[1])
    offset_pos = (position[0] + 0,position[1] + 12)
    offset_pos2 = (position[0] + 0,position[1] + 24)
    offset_pos3 = (position[0] + 0,position[1] + 36)
    cv.putText(img_copy,text, position, cv.FONT_HERSHEY_DUPLEX, 0.6, (30, 30, 255), 1)
    cv.putText(img_copy,subtext, offset_pos, cv.FONT_HERSHEY_DUPLEX, 0.5, (30, 30, 255,0.9), 1)
    cv.putText(img_copy,sub2, offset_pos2, cv.FONT_HERSHEY_DUPLEX, 0.5, (30, 30, 255,0.9), 1)
    cv.putText(img_copy,sub3, offset_pos3, cv.FONT_HERSHEY_DUPLEX, 0.5, (30, 30, 255,0.9), 1)
    #cv.imshow("Annotated frame", annotation_frame)

# def annotate_and_show(img_rgb,berry_list):

#     for berr in berry_list:
#         centroid = berr.
#         if ripeness:
#             text = "ripe"
#         else:
#             text = "overripe"
#         annotation_frame = img_rgb.copy()
#         position = (centroid[0],centroid[1])
#         cv.putText(annotation_frame,text, position, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
#     cv.imshow("Annotated frame", annotation_frame)

def save_annotated_frame(frame, save_dir="/home/blueberryjam/BlueberryJam/img/annotations/"):
    """
    Save an annotated frame to a local location.

    frame: frame to save

    save_dir: directory to which to save frame -- default is img/annotations

    Returns 
    - 0 on success 
    - -1 on failure

    """
    filename = os.path.join(save_dir, f"annotation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
    result = cv.imwrite(filename, frame)
    if result:
        print(f"Image saved as {filename}")
    else:
        print(f"Failed to save image.")
    return 0 if result else -1
