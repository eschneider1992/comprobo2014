#!/usr/bin/env python

import cv2
import pickle
import numpy as np
import sys

D = np.array( [0.08683, -0.28966000000000003, -0.00045000000000000004, -0.00015000000000000001, 0.0])
K = np.array( [[651.38582, 0.0, 327.26766], [0.0, 650.2441, 242.38098],[ 0.0, 0.0, 1.0]])
W = np.array([[0.0, -1.0, 0.0],
			  [1.0, 0.0, 0.0],
			  [0.0, 0.0, 1.0]])

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
pt_num = 0
im1_pts = []
im2_pts = []

def test_epipolar(E,pt1,pt2):
	pt1_h = np.zeros((3,1))
	pt2_h = np.zeros((3,1))
	pt1_h[0:2,0] = pt1.T
	pt2_h[0:2,0] = pt2.T
	pt1_h[2] = 1.0
	pt2_h[2] = 1.0
	return pt2_h.T.dot(E).dot(pt1_h)

def mouse_event(event,x,y,flag,im):
	if event == cv2.EVENT_FLAG_LBUTTON:
		if x < im.shape[1]/2.0:
			l = F.dot(np.array([x,y,1.0]))
			m = -l[0]/l[1]
			b = -l[2]/l[1]
			# equation of the line is y = m*x+b
			y_for_x_min = m*0.0+b
			y_for_x_max = m*(im.shape[1]/2.0-1)+b
			# plot the epipolar line
			cv2.line(im,(int(im.shape[1]/2.0),int(y_for_x_min)),(int(im.shape[1]-1.0),int(y_for_x_max)),(255,0,0))

if __name__ == '__main__':
	im1 = cv2.imread('frame0000.jpg')
	im2 = cv2.imread('frame0001.jpg')

	f = open("correspondences.pickle",'r')
	correspondences = pickle.load(f)
	f.close()

	im1_pts = np.zeros((len(correspondences[0]),2))
	im2_pts = np.zeros((len(correspondences[1]),2))

	im = np.array(np.hstack((im1,im2)))

	# plot the points
	for i in range(len(im1_pts)):
		im1_pts[i,0] = correspondences[0][i][0]
		im1_pts[i,1] = correspondences[0][i][1]
		im2_pts[i,0] = correspondences[1][i][0]
		im2_pts[i,1] = correspondences[1][i][1]

	im1_pts_augmented = np.zeros((1,im1_pts.shape[0],im1_pts.shape[1]))
	im1_pts_augmented[0,:,:] = im1_pts
	im2_pts_augmented = np.zeros((1,im2_pts.shape[0],im2_pts.shape[1]))
	im2_pts_augmented[0,:,:] = im2_pts

	im1_pts_ud = cv2.undistortPoints(im1_pts_augmented,K,D)
	im2_pts_ud = cv2.undistortPoints(im2_pts_augmented,K,D)

	E, mask = cv2.findFundamentalMat(im1_pts_ud,im2_pts_ud,cv2.FM_RANSAC)
	
	F = np.linalg.inv(K.T).dot(E).dot(np.linalg.inv(K))
	
	cv2.imshow("MYWIN",im)
	cv2.setMouseCallback("MYWIN",mouse_event,im)
	while True:
		cv2.imshow("MYWIN",im)
		cv2.waitKey(50)
	cv2.destroyAllWindows()
