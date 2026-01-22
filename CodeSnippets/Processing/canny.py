import cv2

image = cv2.imread("D:/Projects/Uni/Master/bioinspired/CodeSnippets/Processing/grayscaled.png")
canny_image = cv2.Canny(image, 85, 255, image, 3, False)

cv2.imwrite("D:/Projects/Uni/Master/bioinspired/CodeSnippets/Processing/canny_image.png", canny_image)
cv2.imshow("edges", canny_image)
cv2.waitKey(0)
cv2.destroyAllWindows()