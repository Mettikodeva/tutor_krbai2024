import cv2 
import numpy as np

if __name__ == "__main__":

    img = cv2.imread('image.jpg')

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    upper_range = np.array([134,243,237])
    lower_range = np.array([102,181,171])

    mask = cv2.inRange(img_hsv,lower_range,upper_range)
    res = cv2.bitwise_and(img,img,mask=mask)

    MORPH_SIZE = 2
            # create kernel for filter
    element = cv2.getStructuringElement(
                cv2.MORPH_RECT, (2 * MORPH_SIZE, 2 * MORPH_SIZE), (MORPH_SIZE, MORPH_SIZE)
            )
    
    mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, element, iterations=2
            )
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw contours with different color
    
    img_contours = cv2.drawContours(img, contours, -1, (0,255,0), 3)

    # draw bounding box
    for i in range(len(contours)):
        bbox = cv2.boundingRect(contours[i])
        x, y, w, h = bbox

        img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print(f"Bounding box {i}: {x, y, w, h}")
    
    # object position from center of frame
    height, width, _ = img.shape
    for i in range(len(contours)):
        x, y, w, h = cv2.boundingRect(contours[i])
        x_center = x + w / 2
        y_center = y + h / 2
        print(f"Center of bounding box {i}: {x_center, y_center}")
        print(f"Position from center of frame {i}: {x_center - width / 2, y_center - height / 2}")
    
    print(f"Number of contours: {len(contours)}")
    # for i in range(len(contours)):
    #     print(f"Area of contour {i}: {cv2.contourArea(contours[i])}")
    #     print(f"Perimeter of contour {i}: {cv2.arcLength(contours[i], True)}")

    # draw line
    img = cv2.line(img, (width//2, 0), (width//2, height), (0, 255, 0), 2)
    img = cv2.line(img, (0, height//2), (width, height//2), (0, 255, 0), 2)

    # draw line from center of frame to center of bounding box
    for i in range(len(contours)):
        x, y, w, h = cv2.boundingRect(contours[i])
        x_center = x + w / 2
        y_center = y + h / 2
        img = cv2.line(img, (int(width / 2), int(height / 2)), (int(x_center), int(y_center)), (0, 255, 0), 2)

    cv2.imshow('image', img)
    cv2.imshow("hsv", img_hsv)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
    cv2.imshow("contours", img_contours)



    cv2.waitKey(0)
    cv2.destroyAllWindows()
    