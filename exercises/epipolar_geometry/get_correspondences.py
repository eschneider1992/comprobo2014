import cv2
import pickle
import numpy as np

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
scale_factor = 2.0
pt_num = 0
im1_pts = []
im2_pts = []

def mouse_event(event,x,y,flag,im):
	global im1_pts
	global im2_pts
	global pt_num

	if event == cv2.EVENT_FLAG_LBUTTON:
		cv2.circle(im,(x,y),2,colors[pt_num/2 % len(colors)],2)
		cv2.imshow("MYWIN",im)
		if x < im.shape[1]/2.0:
			im1_pts.append((x/scale_factor,y/scale_factor))
		else:
			im2_pts.append(((x - im.shape[1]/2.0)/scale_factor,y/scale_factor))
		pt_num += 1

if __name__ == '__main__':
	im1 = cv2.imread('frame0000.jpg')
	im2 = cv2.imread('frame0001.jpg')
	im1 = cv2.resize(im1,(int(im1.shape[1]*scale_factor),int(im1.shape[0]*scale_factor)))
	im2 = cv2.resize(im2,(int(im2.shape[1]*scale_factor),int(im2.shape[0]*scale_factor)))

	im = np.array(np.hstack((im1,im2)))
	cv2.imshow("MYWIN",im)
	cv2.setMouseCallback("MYWIN",mouse_event,im)
	while True:
		if len(im2_pts) >= 12:
			break
		cv2.waitKey(50)
	f = open('correspondences.pickle','wb')
	pickle.dump((im1_pts,im2_pts),f)
	f.close()
	cv2.destroyAllWindows()