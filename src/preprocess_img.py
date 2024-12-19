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
        if contour_area >= 700 and contour_area < 13000: # only calculate centroids for contours larger than 500px
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
def prune_close_centroids(existing_centroids, thresh=30):
    """
    Prune close centroids until none remain within 'thresh' distance of each other.
    """
    iteration_count = 0
    while True:
        iteration_count += 1
        print(f"[DEBUG][prune_close_centroids] Starting pruning iteration {iteration_count}. Current centroids:")
        for c in existing_centroids:
            print(f"    {c}")

        changed = False
        i = 0
        while i < len(existing_centroids):
            j = i + 1
            while j < len(existing_centroids):
                c1 = existing_centroids[i]
                c2 = existing_centroids[j]
                # Compute Euclidean distance
                dist = ((c1['x'] - c2['x'])**2 + (c1['y'] - c2['y'])**2)**0.5
                print(f"[DEBUG][prune_close_centroids] Checking pair i={i}, j={j}: dist={dist:.2f}, thresh={thresh}")

                if dist < thresh:
                    print(f"[INFO][prune_close_centroids] Found close pair: {c1} and {c2}. Merging...")
                    chosen = compare_centroids(c1, c2)
                    if chosen is c1:
                        print(f"[INFO][prune_close_centroids] Keeping {c1}, removing {c2}")
                        del existing_centroids[j]
                    else:
                        print(f"[INFO][prune_close_centroids] Keeping {c2}, removing {c1}")
                        del existing_centroids[i]
                        i -= 1
                    changed = True
                    print("[DEBUG][prune_close_centroids] A pair was removed. Restarting from scratch.")
                    break
                else:
                    j += 1
            if changed:
                # break from outer loop to restart
                break
            i += 1

        if not changed:
            print(f"[DEBUG][prune_close_centroids] No changes made in iteration {iteration_count}, pruning complete.")
            break


def process_mask_centroids(mask_list, img_rgb):
    processed_imgs = []
    # Instead of storing intermediate results from each mask, we'll store them in a temporary list
    # and only create repackaged_results after final pruning.
    temporary_results = []
    existing_centroids = []
    CLOSE_THRESH = 30

    for i, img in enumerate(mask_list):
        if i > 2:
            continue

        print(f"\n[INFO] Processing mask index {i}")
        processed, raw_centroids = perform_centroiding(img, img_rgb)
        print(f"[DEBUG] Raw centroids for mask {i}: {raw_centroids}")

        curr_mask_centroids = []
        for (cX, cY, box, score) in raw_centroids:
            curr = {
                'x': cX,
                'y': cY,
                'box': box,
                'score': score,
                'mask_idx': i,
                'quality': 1.0,
                'quantity': score
            }
            curr_mask_centroids.append(curr)
        print(f"[DEBUG] Current mask {i} centroids as dict: {curr_mask_centroids}")

        if not existing_centroids:
            # No existing centroids, accept all
            existing_centroids.extend(curr_mask_centroids)
            print(f"[INFO] No existing centroids yet, accepted all from mask {i}.")
        else:
            # Merge new centroids into existing ones
            print(f"[INFO] Merging new centroids from mask {i} into existing centroids.")
            for new_centroid in curr_mask_centroids:
                print(f"\n[DEBUG] Evaluating new centroid: {new_centroid}")

                # Check all close matches
                close_matches = [
                    (idx, ex_cent) for idx, ex_cent in enumerate(existing_centroids)
                    if pythag_centroid_euclid((new_centroid['x'], new_centroid['y']),
                                              [(ex_cent['x'], ex_cent['y'])],
                                              thresh=CLOSE_THRESH)
                ]
                print(f"[DEBUG] Close matches found for new centroid: {close_matches}")

                while close_matches:
                    match_idx, existing_c = close_matches[0]
                    print(f"[DEBUG] About to compare centroids. Existing: {existing_c}, New: {new_centroid}")
                    chosen = compare_centroids(existing_c, new_centroid)
                    if chosen is new_centroid:
                        print(f"[INFO] New centroid won against existing centroid at index {match_idx}. Replacing.")
                        existing_centroids[match_idx] = new_centroid
                        close_matches = [
                            (idx, ex_cent) for idx, ex_cent in enumerate(existing_centroids)
                            if idx != match_idx and pythag_centroid_euclid(
                                (new_centroid['x'], new_centroid['y']),
                                [(ex_cent['x'], ex_cent['y'])],
                                thresh=CLOSE_THRESH)
                        ]
                        print(f"[DEBUG] Re-checked close matches after merge: {close_matches}")
                    else:
                        print(f"[INFO] Existing centroid at index {match_idx} won. Discarding new centroid.")
                        new_centroid = chosen
                        break
                else:
                    # If loop completes normally, new_centroid survived
                    if new_centroid not in existing_centroids:
                        print(f"[INFO] Adding surviving new centroid to existing.")
                        existing_centroids.append(new_centroid)

        print(f"[DEBUG] Existing centroids after processing mask {i}: {existing_centroids}")
        # Store the current mask's raw centroids temporarily
        temporary_results.append(curr_mask_centroids)
        processed_imgs.append(processed)

    # After processing all masks, perform pruning
    print("\n[INFO] Final pruning pass over existing centroids.")
    prune_close_centroids(existing_centroids, CLOSE_THRESH)
    print(f"[DEBUG] Existing centroids after final pruning: {existing_centroids}")

    # Now repackage results from the final, pruned existing_centroids only
    # so that repackaged_results contain no duplicates.
    # We do not use temporary_results directly anymore. Instead, create a single final result list
    # from the pruned existing_centroids.
    repackaged_results = [[]]  # We'll put all final centroids into the first list for consistency
    for c in existing_centroids:
        repackaged_results[0].append([
            c['x'],
            c['y'],
            c['box'],
            c['score'],
            c['mask_idx'],
            c['quality'],
            c['quantity']
        ])

    print("[INFO] Finished process_mask_centroids.")
    print(f"[DEBUG] Final repackaged results: {repackaged_results}")
    print(f"[DEBUG] Final existing centroids: {existing_centroids}")

    return processed_imgs, repackaged_results, existing_centroids


def compare_centroids(old_centroid, new_centroid):
    """
    Compare two centroids based on their similarity scores and mask indices.
    Return the centroid dictionary that should be kept.
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

    if new_priority > old_priority:
        # New centroid from a higher priority mask
        if old_priority > 1.3 * new_priority:
            return old_centroid
        return new_centroid
    elif new_priority < old_priority:
        # Old centroid from a higher priority mask
        if new_priority > 1.3 * old_priority:
            return new_centroid
        return old_centroid
    else:
        # Same priority, choose by higher score
        return new_centroid if new_centroid['score'] > old_centroid['score'] else old_centroid

