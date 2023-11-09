import numpy as np
import cv2 as cv

def undistortCamera():
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = ['calibrationPics\WIN_20231109_14_38_01_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_59_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_56_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_52_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_48_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_45_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_38_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_35_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_24_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_19_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_07_Pro.jpg',
    'calibrationPics\WIN_20231109_14_37_04_Pro.jpg',]
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv.drawChessboardCorners(img, (9,6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)
    cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    img = cv.imread('calibrationPics\WIN_20231109_14_37_07_Pro.jpg')
    h,  w = img.shape[:2]
    newcameramtx, roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv.imwrite('calibresult.png',dst)

    cv.imshow('original', cv.imread('calibrationPics\WIN_20231109_14_37_07_Pro.jpg'))
    cv.imshow('fixed', dst)
    cv.waitKey(50000)

    cv.destroyAllWindows()