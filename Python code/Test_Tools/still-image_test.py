import cv2 as cv 

cap = cv.VideoCapture(0)
ret, frame = cap.read()
if ret:
    cv.imwrite('imagetest.jpeg', frame)
cap.release()