import cv2 as cv
import numpy as np
from matplotlib import colors
from random import randint
import math
# IMG_PATH = 'R_06.11'
# MASK = [(10, 70, 100), (250, 130, 250)] # [low, high]

def preprocess_image( mask:list, path:str = None,frame = None):

    if path is not None:
        # load image
        #/home/blueberryjam/BlueberryJam/img/test_set/ripe/R_06.14.jpg
        full_path = '/home/blueberryjam/BlueberryJam/img/test_set/ripe/'+ path +'.jpg'
        img = cv.imread(full_path)
    elif frame is not None:
        img = frame #convert_pc2_cv2img(frame)
    else:
        # print("ERROR: no path or frame provided to preprocessor")
        raise RuntimeError

    if img is None:
        raise FileNotFoundError(f"Could not load image at{full_path}")

    # print("processing got here 1")

# convert to viewable and usable color spaces
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    img_luv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # cv.imshow("RGB Image", img_rgb)
    # cv.waitKey(0)

    # print("processing got here 2")
    
# normalize colors (may be useless)
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    pixel_colors = img_luv.reshape((np.shape(img_luv)[0]*np.shape(img_luv)[1], 3))
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
# define color mask
    luv_lo = mask[0]
    luv_hi = mask[1]
# apply color mask to image
    mask_luv = cv.inRange(img_luv, luv_lo, luv_hi)
    # cv.imshow("LUV Mask", mask_luv)
    # cv.waitKey(0)

    # print("processing got here 3")
# threshold masked image to get binary image
    retval,thresh = cv.threshold(mask_luv,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
    cv.imshow("Binary Image", thresh)
    #cv.waitKey(0)

    # print("processing got here 4")
# find contours
    contours, _ = cv.findContours(thresh,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# find centroids for each contour in image
    for contour in contours:
        # print(cv.contourArea(contour))
        
        if cv.contourArea(contour) <= 500: # only calculate centroids for contours larger than 500px
# calculate moments of the contour
            M = cv.moments(contour)
            if M["m00"] != 0:  # avoid divide by zero error
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
# put the centroid on the RGB image
                cv.circle(img_rgb, (cX, cY), 5, (255, 0, 0), -1)  # Blue dot at the centroid
                cv.putText(img_rgb, "Centroid", (cX - 20, cY - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

# show the result with centroids marked
    cv.imshow("Centroids in RGB Image", img_rgb)
    #cv.waitKey(0)
    # print("processing got here 5")

    return img_rgb

def convert_pc2_cv2img(frame):
    return cv.cvtColor(frame,cv.COLOR_RGB2BGR)

def classify_berry_naive(frame,berry):
    #preprocessing is complete
    berry.ripeness = randint(-1,2)
    return berry


# preprocess_image(IMG_PATH, MASK)

def perform_backgrounding( mask:list, path:str = None,frame = None):

    if path is not None:
        # load image
        #/home/blueberryjam/BlueberryJam/img/test_set/ripe/R_06.14.jpg
        full_path = '/home/blueberryjam/BlueberryJam/' + path
        img = cv.imread(full_path)
    elif frame is not None:
        img = frame #convert_pc2_cv2img(frame)
    else:
        # print("ERROR: no path or frame provided to preprocessor")
        raise RuntimeError

    if img is None:
        raise FileNotFoundError(f"Could not load image at{full_path}")

    # print("processing got here 1")

# convert to viewable and usable color spaces
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    img_luv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # cv.imshow("RGB Image", img_rgb)
    # cv.waitKey(0)

    # print("processing got here 2")
    
# normalize colors (may be useless)
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    pixel_colors = img_luv.reshape((np.shape(img_luv)[0]*np.shape(img_luv)[1], 3))
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
# define color mask
    luv_lo = mask[0]
    luv_hi = mask[1]
# apply color mask to image
    mask_luv = cv.inRange(img_luv, luv_lo, luv_hi)
    #cv.imshow("HSV Mask", mask_luv)
    #cv.waitKey(0)

    return mask_luv,img_rgb

def perform_centroiding(masked_frame,img_rgb):
    # threshold masked image to get binary image
    retval,thresh = cv.threshold(masked_frame,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
    # cv.imshow("Binary Image", thresh)
    # #cv.waitKey(0)

    # print("processing got here 4")
# find contours
    contours, _ = cv.findContours(thresh,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# find centroids for each contour in image
    centroids = []
    for contour in contours:
        # print(cv.contourArea(contour))
        contour_area = cv.contourArea(contour) #original just asks contour to be larger than 500
        if contour_area >= 400 and contour_area < 13000: # only calculate centroids for contours larger than 500px
# calculate moments of the contour
            M = cv.moments(contour)
            if M["m00"] != 0:  # avoid divide by zero error
                cX = int(M["m10"] / M["m00"]) 
                cY = int(M["m01"] / M["m00"])
# put the centroid on the RGB image  

                (circle_x,circle_y), radius = cv.minEnclosingCircle(contour)

                x_min = int(cX - radius) if int(cX - radius) > 0 else 0 
                y_min = int(cY - radius) if int(cY - radius) > 0 else 0

                if radius > 70:
                    continue #radius too big

                # if int(cX - radius) > 0 or int(cY - radius) > 0:
                #     continue
                width = int(2*radius)
                height = width
                box = [x_min,y_min,width,height]

                score = contour_area/ (math.pi * radius*radius)

                # cv.circle(img_rgb, (cX, cY), 5, (255, 0, 0), -1)  # Blue dot at the centroid
                # cv.putText(img_rgb, "Centroid", (cX - 20, cY - 10), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1)
                # cv.rectangle(img_rgb,(x_min,y_min),(x_min + width,y_min + height),(0,255,0),2)

                centroids.append((cX,cY,box,score))

# show the result with centroids marked
    # cv.imshow("Centroids in RGB Image", img_rgb)
    # #cv.waitKey(0)
    # # print("processing got here 5")

    return img_rgb,centroids
def pythag_centroid(centroid,centroid_list,thresh):
    #thresh is a number of pixels
    # print(f"analyzing centroid {centroid}")
    for cent in centroid_list:
        # print(centroid)
        # print(cent)
        cond1 = centroid[0] < cent[0] + thresh
        cond2 = centroid[0] > cent[0] - thresh
        cond3 = centroid[1] < cent[1] + thresh
        cond4 = centroid[1] > cent[1] - thresh

        #print(cond1,cond2,cond3,cond4)
        if cond1 and cond2 and cond3 and cond4:
            return True
    return False

def pythag_centroid_euclid(centroid, centroid_list, thresh):
    cx, cy = centroid
    for (ex, ey) in centroid_list:
        distance = math.sqrt((cx - ex)**2 + (cy - ey)**2)
        if distance < thresh:
            return True
    return False
def crop_img_center(img_rgb, crop_width,bias=0):
# get centroid'd image in RGB
# convert to LUV


    height,width = img_rgb.shape[:2]

    band_height = height
    band_width = crop_width

    start_y = (height - band_height) //2
    end_y = start_y + band_height
    start_x = (width - band_width) //2 + bias
    end_x = start_x + band_width
    return img_rgb[start_y:end_y, start_x:end_x]

def apply_multi_bg(masks: list, path: str = None, frame = None) -> list:
    '''
    Applies multiple masks to an image, returning a list of masked images.
        Args: 
    
    masks: a list of tuples containing the low and high values for L, U, and V color channels
    
    path: a string representing the relative path to the image to be masked; default value is None

    frame: a cropped frame from a video; default value is None

        Returns:
    
        list of masked images # index -1 will always be the image of all applied masks combined
    '''
    # Initialize an empty list to store the masked images
    masked_images = []
    mask_list = []

    # Loop through each range of low and high values, and use the apply_background function
    for i, (lower, upper) in enumerate(masks):
        # Call the apply_background function to create the mask for each range
        mask, img_rgb = perform_backgrounding([lower, upper], path=path, frame=frame)

        # Apply the mask using cv2.bitwise_and
        masked_image = cv.bitwise_and(img_rgb, img_rgb, mask=mask)

        # Append the masked image to the list
        masked_images.append(masked_image)
        mask_list.append(mask)

        # Initialize the combined_mask with the first mask on the first iteration
        if i == 0:
            combined_mask = np.zeros_like(mask)  # Initialize the combined mask with the first mask's shape

        # Combine all the masks using bitwise OR
        combined_mask = cv.bitwise_or(combined_mask, mask)

    # Apply the combined mask to the image
    combined_image = cv.bitwise_and(img_rgb, img_rgb, mask=combined_mask)
    mask_list.append(combined_mask)

    # Append the combined image to the list
    masked_images.append(combined_image)

    return masked_images, img_rgb,mask_list

#a replacement for a large block of code in main
def process_mask_centroids(mask_list, img_rgb):
    processed_imgs = []
    centroid_results = []
    existing_centroids = []

    for i, img in enumerate(mask_list):
        if i >2:
            continue
        processed, raw_centroids = perform_centroiding(img, img_rgb)
        print(f"raw centroids: {raw_centroids}")

        # Convert returned centroids into a structured form
        curr_mask_centroids = []
        for (cX, cY, box, score) in raw_centroids:
            curr_mask_centroids.append({
                'x': cX,
                'y': cY,
                'box': box,
                'score': score,
                'mask_idx': i,
                'quality': 1.0,
                'quantity': score
            })
        
        print(f"curr_mask_centroids: {curr_mask_centroids}")

        if existing_centroids:
            # We'll store newly accepted centroids that do not conflict
            accepted_new_centroids = []
            for new_centroid in curr_mask_centroids:
                print(f"new centroid: {new_centroid}")
                # Check if it's close to any existing centroid
                close_matches = [
                    (idx, ex_cent) for idx, ex_cent in enumerate(existing_centroids)
                    if pythag_centroid_euclid((new_centroid['x'], new_centroid['y']),
                                              [(ex_cent['x'], ex_cent['y'])],
                                              thresh=30)
                ]
                print(f"close_matches: {close_matches}")
                if not close_matches:
                    # No close existing centroid, accept it
                    accepted_new_centroids.append(new_centroid)
                else:
                    # There's at least one close existing centroid
                    # For simplicity, handle just the first match
                    match_idx, existing_c = close_matches[0]
                    print(f"about to compare centroids { existing_c},{new_centroid}")
                    chosen = compare_centroids(existing_c, new_centroid)
                    if chosen is new_centroid:
                        # Replace the old centroid with the new one
                        existing_centroids[match_idx] = new_centroid
                    # If chosen is the old one, discard the new centroid
            print(f"accepted new centroids {accepted_new_centroids}")

            # Add all newly accepted centroids that didn't overlap or lost replacement
            for c in accepted_new_centroids:
                if c not in existing_centroids:
                    existing_centroids.append(c)
        else:
            # No existing centroids yet, so accept all current centroids
            existing_centroids.extend(curr_mask_centroids)

        centroid_results.append(curr_mask_centroids)
        processed_imgs.append(processed)
    
    # Repackage centroid_results from dicts to lists
    # Structure: [x,y,box,score,mask_idx,quality,quantity]
    repackaged_results = []
    for centroid_list in centroid_results:
        repackaged_list = []
        for c in centroid_list:
            repackaged_list.append([
                c['x'],
                c['y'],
                c['box'],
                c['score'],
                c['mask_idx'],
                c['quality'],
                c['quantity']
            ])
        repackaged_results.append(repackaged_list)

    print(f"repacked results: {repackaged_results}")


    return processed_imgs, repackaged_results, existing_centroids

def compare_centroids(old_centroid, new_centroid):
    """
    Compare two centroids based on their similarity scores and mask indices.
    Return the centroid dictionary that should be kept.
    Example logic:
    1. Prioritize mask index (0 highest priority, then 1, then 2).
    2. If both centroids have the same priority, choose by similarity score.
    """

    mask_priority = {0: 3, 1: 2, 2: 1}  # Overripe=0 (highest), Ripe=1, Underripe=2 (lowest)

    old_priority = mask_priority[old_centroid['mask_idx']]
    new_priority = mask_priority[new_centroid['mask_idx']]

    # Update quantities and qualities
    new_quantity = old_centroid['quantity'] + new_centroid['quantity']
    old_centroid['quantity'] = new_quantity
    new_centroid['quantity'] = new_quantity

    new_centroid['quality'] = new_centroid['score'] / new_quantity
    old_centroid['quality'] = old_centroid['score'] / new_quantity

    # Priority logic
    if new_priority > old_priority:
        # New centroid is from a higher priority mask
        if old_priority > 1.3 * new_priority:
            # Keep old centroid despite the difference if this condition is met
            return old_centroid
        return new_centroid
    elif new_priority < old_priority:
        # Old centroid is from a higher priority mask
        # Check special condition
        if new_priority > 1.3 * old_priority:
            # Keep new centroid since it surpasses old centroid by a significant margin
            return new_centroid
        return old_centroid
    else:
        # Same priority, choose by similarity score
        return new_centroid if new_centroid['score'] > old_centroid['score'] else old_centroid
