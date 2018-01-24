This is my submission for Visual Understanding coursework 1.

Please follow the instructions below to succesfully run and test my implementations of the various parts
of the coursework. Since the coursework pdf provided started numbering the various parts from 2.1 (Convolution),
then my submission also follows this format.

To run the code, type the following commands in MATLAB command window, while in the correct directory:

Part 2 - Convolutions:
	Subpart 2.1 - Basic Convolution: 								>> cwp2(1)
	Subpart 2.2 - Extended Convolution: 							>> cwp2(2)
	Subpart 2.3 - Gaussian kernel and hor,ver,diag, unsharp:		>> cwp2(3)
	Subpart 3.4 - FFT Convolution and time vs kernel area graph:	>> cwp2(4)
	
Part 3 - Features:
	Subpart 3.1 - Canny Edges:										>> cwp3(1,hsize,sigma)
		where hsize is the side of the square Gaussian kernel
		and sigma is the standard deviation to be used:	recommended >> cwp3(1,5,1.7)
	Subpart 3.2 - Harris Corners:									>> cwp3(2,hsize,sigma)
		where hsize is the side of the square Gaussian kernel
		and sigma is the standard deviation to be used:	recommended >> cwp3(2,7,1.4)
	Subpart 3.3 - Difference of Gaussians: 							>> cwp3(3,1,1)
		This part is unfinished but will return the extrema points of the DoG. 
	
Part 4 - Matching: 
	For every subpart there is a commented section at the top of the file. Please uncomment 
	the images corresponding to the subpart to see the optimal resuts.
	Subpart 4.1 - Brute Force Matching:								>> cwp4(1)
	Subpart 4.2 - Homography estimation:							>> cwp4(2)
	Subpart 4.3 - Visualising the homography:						>> cwp4(3)
	Subpart 4.4 - Ransac Homography Estimation:						>> cwp4(4)
	
Part 5 - Intrinsic parameters:
	By default, the function wil use 5 images to estimate the intrinsic matrix and
	radial distortion. In order to change that please change the variable n_images and
	the add or subtract indices in the index array at the top of the file.
	To run this part: 												>> cwp5()

Part 6 - Fundamental and Essential matrix:
	Subpart 6.1 - Normalised 8-Point algorithm:						>> cwp6(1)
	Subpart 6.2 - Ransac Fundamental matrix estimation:				>> cwp6(2)
	Subpart 6.3 - Essential Matrix and decomposition:				>> cwp6(3)
	Visualisation for part 6.3 is provided with part 7.
	
Part 7 - Triangulation and reconstruction:
	This part is included in the cwp6.m file as the 4th subpart therefore in order
	to run it please type the following:
	Subpart 7.1 - Triangulation and point cloud:					>> cwp6(4)
	Sorry for the confusion.
	
	
Thank you very much.