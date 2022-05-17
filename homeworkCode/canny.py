import cv2
# from cv2 import *

# Read the original image
img = cv2.imread('imagen_test.jpeg')
# Display the original image
cv2.imshow('Original', img)
cv2.waitKey(0)

# Convert grayscale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Blur the image for better edge detection
img_blur = cv2.GaussianBlur(img_gray, (3,3),0)

# Sobel edge detection
sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) #Sobel edge detection on the x axis
sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) #Sobel edge detection on the y axis
sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) #Combined axis sobel edge detection

# Display sobel edge detection images
cv2.imshow('Sobel X', sobelx)
cv2.waitKey(0)
cv2.imshow('Sobel Y', sobely)
cv2.waitKey(0)
cv2.imshow('Sobel X Y', sobelxy)
cv2.waitKey(0)

# Canny edge detection
edges = cv2.Canny(image = img_blur, threshold1 = 100, threshold2 = 100) #Canny edge detection

# Display Canny Edge Detection
cv2.imshow('Canny edge detection', edges)
cv2.waitKey(0)

cv2.destroyAllWindows()