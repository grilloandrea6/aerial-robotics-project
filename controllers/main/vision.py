import cv2
import numpy as np


# Function to detect pink squares
def detect_pink_square(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for pink color
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([180, 255, 255])
    
    # Threshold the HSV image to get only pink colors
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Iterate through detected contours
    for contour in contours:
        # Approximate the contour to a polygon
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        area = cv2.contourArea(contour)
        print(f"area {area}")
        # If the polygon has 4 vertices, it's likely a square
        if len(approx) == 4 and area > 400:
            print("have an interesting contour")

            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  #add this line

            M = cv2.moments(contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


            mask = cv2.circle(mask, center, 4, (0,0,255), -10)
            

            cv2.drawContours(mask, [contour], 0, (255,0,0), 3)
            mask = cv2.circle(mask, (5,5), 4, (0,0,255), -10)
            mask = cv2.circle(mask, (295,295), 4, (255,0,255), -10)

            mask = cv2.circle(mask, (295,5), 4, (255,0,0), -10)

            cv2.imshow("mask",mask)
            return (center[0]-150,center[1]-150)

            #return contour
        else:
            print("not interesting contour")
    # print(frame.shape)
    # print(mask.shape)

    
    return None




# Load the PNG image
image = cv2.imread('camera2.png')

cont = detect_pink_square(image)

M = cv2.moments(cont)
#center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

print(cont)
#print(f"center {center}")

# # Check if the image was loaded successfully
# if image is None:
#     print("Error: Unable to load image.")
# else:
#     # Display or process the image as needed
#     cv2.imshow('PNG Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
