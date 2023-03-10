import cv2 as cv

cap = cv.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while True:
    ret, frame = cap.read()

    cv.imshow("frame",frame)
    k = cv.waitKey(5)
    if k == 27:
        break
cap.release()
cv.destroyAllWindows
