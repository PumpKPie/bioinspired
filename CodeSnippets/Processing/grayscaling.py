import cv2

image = cv2.imread("D:/Projects/Uni/Master/bioinspired/CodeSnippets/Processing/sample.png")
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imwrite("D:/Projects/Uni/Master/bioinspired/CodeSnippets/Processing/grayscaled.png", gray_image)
cv2.imshow("Grayscale", gray_image)
cv2.waitKey(0)
cv2.destroyAllWindows()