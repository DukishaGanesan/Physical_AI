import cv2
print(cv2.__version__)

# Test GUI
cv2.imshow('Test', cv2.imread('test_image.jpeg'))
cv2.waitKey(0)
cv2.destroyAllWindows()
