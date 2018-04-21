import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
#def color_thresh(img, rgb_thresh=(180, 170, 160)):
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    color_select = np.zeros_like(img[:,:,0])
    # TODO:  add above black to remove out of scope region
    below_thresh = (img[:,:,0] <= rgb_thresh[0]) | (img[:,:,1] <= rgb_thresh[1]) | (img[:,:,2] <= rgb_thresh[2])
    color_select[below_thresh] = 1
    return color_select

def gold_thresh(img):
    color_select = np.zeros_like(img[:,:,0])
    thresh = (img[:,:,0] > 100) & (img[:,:,1] > 100) & (img[:,:,2] < 80)
    color_select[thresh] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    voffset = 5
    hoffset = 5
    source = np.float32([[120,96],[200,96],[14,140],[300,140]])
    destination = np.float32([[Rover.img.shape[1]/2-hoffset-5, Rover.img.shape[0]-voffset-10],
                              [Rover.img.shape[1]/2-hoffset+5, Rover.img.shape[0]-voffset-10],
                              [Rover.img.shape[1]/2-hoffset-5, Rover.img.shape[0]-voffset],
                              [Rover.img.shape[1]/2-hoffset+5, Rover.img.shape[0]-voffset]])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    #source = np.float32([[120,16],[200,16],[14,60],[300,60]])
    #destination = np.float32([[155,64],[165,64],[155,74],[165,74]])
    #warped = perspect_transform(Rover.img[80:,:], source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # scale of warped image has changed such that about 30x30 pixels represent the visible
    # range in front of rover:  dont' try to use the full 160x320 on the 200x200 world map
    #kernel = np.ones((3,3),np.uint8)
    #warped_eroded = cv2.erode(warped,kernel,iterations=1)
    
    nav_threshed = color_thresh(warped)
    obstacle_threshed = obstacle_thresh(warped)
    rock_threshed = gold_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    #Rover.vision_image = warped
    Rover.vision_image[:,:,0] = np.multiply(obstacle_threshed,192)
    Rover.vision_image[:,:,1] = np.multiply(rock_threshed,192)
    Rover.vision_image[:,:,2] = np.multiply(nav_threshed,192)

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw

    print("Rover xpos {} ypos {} yaw {}".format(int(xpos),int(ypos),Rover.yaw))

    pitch = Rover.pitch
    if pitch > 358.0:
        pitch -= 360.0
    roll = Rover.roll
    if roll > 358.0:
        roll -= 360.0
    if abs(pitch) < 2.0 and abs(roll) < 2.0:     
        #xpix,ypix = rover_coords(obstacle_threshed[135:,140:170])
        xpix,ypix = rover_coords(obstacle_threshed[125:,:])
        xpix_world,ypix_world = pix_to_world(xpix,ypix,xpos,ypos,yaw,200,10)
        Rover.worldmap[ypix_world,xpix_world,0] += 1
    
        xpix,ypix = rover_coords(rock_threshed[125:,130:180])
        xpix_world,ypix_world = pix_to_world(xpix,ypix,xpos,ypos,yaw,200,10)
        Rover.worldmap[ypix_world,xpix_world,1] = 128
        Rover.searchmap[ypix_world,xpix_world,0] = 1  # mark for searching (gold)

        if len(ypix_world) > 0:
            print('marked gold in {} pixels'.format(len(ypix_world)))

        xpix,ypix = rover_coords(nav_threshed[125:,130:180])
        #xpix,ypix = rover_coords(nav_threshed[125:,:])
        xpix_world,ypix_world = pix_to_world(xpix,ypix,xpos,ypos,yaw,200,10)
        Rover.worldmap[ypix_world,xpix_world,2] = 128
        Rover.searchmap[ypix_world,xpix_world,1] = 1  # mark for searching (navigable area)
        #Rover.worldmap[ypix_world,xpix_world,0] = 0  # TEST

    xpix,ypix = rover_coords(nav_threshed)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dists,angles = to_polar_coords(xpix,ypix)
    Rover.nav_dists = dists
    Rover.nav_angles = angles
    
    
    return Rover
