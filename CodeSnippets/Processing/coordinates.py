import cv2
import numpy as np

edges = cv2.imread('D:/Projects/Uni/Master/Disertatie/Code/Assets/canny_image.png', 0)

contours, hierarchy = cv2.findContours(
    edges, 
    cv2.RETR_LIST, # RETR_EXTERNAL gets only the outer contours.
    cv2.CHAIN_APPROX_NONE # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and 
) #diagonal segments and leaves only their end points. CHAIN_APPROX_NONE to get ALL points.


canvas = np.zeros_like((edges), dtype=np.uint8) # 200x200 3-channel (color) image

# 2. Draw all contours
# Arguments: Image, Contours list, Contour index (-1 means all), Color (B,G,R), Thickness
cv2.drawContours(canvas, contours, -1, (255, 255, 255), 3) # Draw in Green, thickness 2
cv2.imshow('output', canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()