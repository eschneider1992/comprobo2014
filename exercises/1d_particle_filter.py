import numpy as np
import cv2
from math import pi, sqrt, e

def draw_random_sample(choices, probabilities, n):
	""" Return a random sample of n elements from the set choices with the specified probabilities
		choices: the values to sample from represented as a list
		probabilities: the probability of selecting each element in choices represented as a list
		n: the number of samples
	"""
	values = np.array(range(len(choices)))
	probs = np.array(probabilities)
	bins = np.add.accumulate(probs)
	inds = values[np.digitize(np.random.random_sample(n), bins)]
	samples = []
	for i in inds:
		samples.append(choices[int(i)])
	return samples

cv2.namedWindow("my_win")

LEFT_ARROW = 65361
RIGHT_ARROW = 65363
measurement_noise = .5
movement_noise = .05
n = 100
particles = np.random.normal(size=(n,))
w = [1.0/n]*n
x = 0.0

while True:
	im = np.zeros((500,500,3))
	key = cv2.waitKey(50)
	movement = 0.0
	if key == LEFT_ARROW:
		movement = -.1
	elif key == RIGHT_ARROW:
		movement = .1
	if movement:
		x += movement + np.random.normal(scale=movement_noise)
		z = x + np.random.normal(scale=measurement_noise)
		for i in range(len(particles)):
			particles[i] += movement + np.random.normal(scale=movement_noise)
			w[i] = 1/(measurement_noise*sqrt(2*pi))*e**(-(particles[i]-z)**2/(2*measurement_noise**2))
		w = [weight / sum(w) for weight in w]
		particles = draw_random_sample(particles,w,n)
		w = [1.0/n]*n

	# visualize the state of the particles
	for p in particles:
		cv2.circle(im, (int(p*100+im.shape[0]/2.0), 250), 5,(255,0,0))
	cv2.circle(im, (int(x*100+im.shape[0]/2.0), 250), 5,(0,255,0),2)
	if movement:
		cv2.circle(im, (int(z*100+im.shape[0]/2.0), 250), 5,(0,0,255),2)

	cv2.imshow("my_win",im)