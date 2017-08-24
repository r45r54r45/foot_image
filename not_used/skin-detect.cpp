#include "skin-detect.h"

int main(int argc, char** argv)
{
	if (argc > 2)
	{
		cout << "Usage" << argv[0] << " /path/to/video for file input" << endl;
		cout << "or" << endl;
		cout << "Usage" << argv[0] << " for webcam input" << endl;
		return -1;
	}
	else if (argc == 2)
	{
		cap.open(argv[1]);
	}
	else
	{
		cap.open(CV_CAP_ANY);
	}
	if (!cap.isOpened())   //Check if video has been opened correctly
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}
	//for (int i=0; i<3000; i++) { //Skip to interesting parts of long videos
	cap >> img; //Capture the first frame of the video
	//}
	if (!img.data)  //Check for invalid input
	{
		cout <<  "Could not open video" << endl ;
		return -1;
	}
	imgHist = Mat::zeros(img.size(), CV_32F);
	namedWindow(window1_name, CV_WINDOW_AUTOSIZE);  //Create video window
	namedWindow(window2_name, CV_WINDOW_AUTOSIZE);  //Create thresholded video window
	namedWindow(window3_name, CV_WINDOW_AUTOSIZE);	//Create settings window
	createTrackbar("Colour Segmentation", window3_name, &cs, 1, process_image);
	createTrackbar("Min Cr", window3_name, &min_Cr, 255, process_image);
	createTrackbar("Max Cr", window3_name, &max_Cr, 255, process_image);
	createTrackbar("Min Cb", window3_name, &min_Cb, 255, process_image);
	createTrackbar("Max Cb", window3_name, &max_Cb, 255, process_image);
	createTrackbar("Density Regularisation", window3_name, &dr, 1, process_image);
	createTrackbar("Erosion 1 Kernel Size", window3_name, &erode1, 9, process_image);
	createTrackbar("Dilation 1 Kernel Size", window3_name, &dilate1, 9, process_image);
	createTrackbar("Luminance Regularisation", window3_name, &lr, 1, process_image);
	createTrackbar("stddev", window3_name, &deviation, 10, process_image);
	createTrackbar("Geometric Correction", window3_name, &gc, 1, process_image);
	createTrackbar("Erosion 2 Kernel Size", window3_name, &erode2, 9, process_image);
	createTrackbar("Dilation 2 Kernel Size", window3_name, &dilate2, 9, process_image);
	createTrackbar("Average Frames", window3_name, &af, 1, process_image);
	createTrackbar("Weighting", window3_name, &weighting, 255, process_image);
	if (argc == 2)
	{
		while (true)
		{
			c = waitKey(20);
			if ((char)c == 27) break; //Exit if ESC key is hit
			//Debug info
			cout << "Frames in video: " << cap.get(CV_CAP_PROP_FRAME_COUNT) << "	Current Frame: " << cap.get(CV_CAP_PROP_POS_FRAMES) << "	FPS: " << cap.get(CV_CAP_PROP_FPS) << endl;
			if (cap.get(CV_CAP_PROP_FRAME_COUNT) == cap.get(CV_CAP_PROP_POS_FRAMES)) cap.set(CV_CAP_PROP_POS_FRAMES, 0);  //If at end of video, loop back to start
			process_image(0, 0);
		}
	}
	else
	{
		while (true)
		{
			c = waitKey(20);
			if ((char)c == 27) break; //Exit if ESC key is hit
			process_image(0, 0);
		}
	}
}

void process_image(int, void*)
{
	imshow(window1_name, img); //Show captured frame
	cvtColor(img, img, CV_BGR2YCrCb); //Convert to YUV colorspace
	if (cs) colour_segmentation();
	if (dr) density_regularisation();
	if (lr) luminance_regularisation();
	if (gc) geometric_correction();
	if (af) average_frame();
	imshow(window2_name, imgHist);
	cap >> img; //Capture next frame
}

/***********************************************************************
 * Step 1: Colour Segmentation
 *
 * Classify pixels as either skin or non-skin. Skin tones fall within a
 * small range of Chrominance values:
 * Cr = 133 to 173
 * Cb = 77 to 127
 **********************************************************************/

void colour_segmentation()
{
	inRange(img, Scalar(0, min_Cr, min_Cb), Scalar(255, max_Cr, max_Cb), imgFilter);
}

/***********************************************************************
 * Step 2: Density Regularisation
 *
 * Colour segmentation produces a very noisy image with a lot of false
 * positives. We therefore need to find areas with a high probability of
 * that belong to the facial region. Group pixels in 4x4 clusters and
 * find the sum of skin coloured pixels in each cluster.
 *
 * Classify each cluster as follows to achieve a density map:
 * 0 <= sum < 16; noise or non-facial region
 * sum = 16; facial region
 *
 * Following this perform morphology as below:
 * 1. discard all clusters at the perimeter of the frame
 * 2. Erode any full-density point if it is surrounded by less than ﬁve
 * 	  other full-density points in its local 3x3 neighborhood
 * 3. Dilate any point of either zero or intermediate density if there
 *    are more than two full-density points in its local 3x3
 *    neighborhood
 **********************************************************************/
void density_regularisation()
{
	Mat sum;
	sum = Mat::zeros(img.rows, img.cols, CV_8UC1);
	uchar op;
	int erode, dilate;
	for (int i = 0; i < img.rows; i += 4) //Cycle over horizontal clusters
	{
		for (int j = 0; j < img.cols; j += 4) //Cycle over vertical clusters
		{
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					if (imgFilter.at<uchar>(i + k, j + l) != 0) sum.at<uchar>(i, j)++;
				}
			}
			if (sum.at<uchar>(i, j) == 0 || i == 0 || j == 0 || i == (img.rows - 4) || j == (img.cols - 4)) op = 0;
			else if (sum.at<uchar>(i, j) > 0 &&  sum.at<uchar>(i, j) < 16) op = 128;
			else op = 255;
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					imgFilter.at<uchar>(i + k, j + l) = op;
				}
			}
		}
	}
	for (int i = 4; i < (img.rows - 4); i += 4) //Cycle over horizontal clusters
	{
		for (int j = 4; j < (img.cols -4); j += 4) //Cycle over vertical clusters
		{
			erode = 0;
			if (imgFilter.at<uchar>(i, j) == 255)
			{
				for (int k = -4; k < 5; k += 4)
				{
					for (int l = -4; l < 5; l += 4)
					{
						if (imgFilter.at<uchar>(i + k, j + l) == 255) erode++;
					}
				}
				if (erode < erode1)
				{
					for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
					{
						for (int l = 0; l < 4; l++) //Cycle vertically within cluster
						{
							imgFilter.at<uchar>(i + k, j + l) = 0;
						}
					}
				}
			}
		}
	}
	for (int i = 4; i < (img.rows - 4); i += 4) //Cycle over horizontal clusters
	{
		for (int j = 4; j < (img.cols - 4); j += 4) //Cycle over vertical clusters
		{
			dilate = 0;
			if (imgFilter.at<uchar>(i, j) < 255)
			{
				for (int k = -4; k < 5; k += 4)
				{
					for (int l = -4; l < 5; l += 4)
					{
						if (imgFilter.at<uchar>(i + k, j + l) == 255) dilate++;
					}
				}
				if (dilate > dilate1)
				{
					for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
					{
						for (int l = 0; l < 4; l++) //Cycle vertically within cluster
						{
							imgFilter.at<uchar>(i + k, j + l) = 255;
						}
					}
				}
			}
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					if (imgFilter.at<uchar>(i + k, j + l) != 255) imgFilter.at<uchar>(i + k, j + l) = 0;
				}
			}
		}
	}
}

/***********************************************************************
 * Step 3: Luminance Regularisation
 *
 * In a typical teleconference the brightness is nonuniform throughout
 * the facial region, while the background region tends to have a more
 * even distribution of brightness. Hence, based on this characteristic,
 * background region that was previously detected due to its skin-color
 * appearance can be further eliminated
 *
 * Calculate the standard deviation of the luminance in an 8x8 pixel
 * area (under 4:2:0 subsampling).
 * if (stddev >= 2 && step2value == 1) step3value =1;
 **********************************************************************/

void luminance_regularisation()
{
	float xbar, sse, stddev;
	int op;
	
	for (int i = 4; i < (img.rows - 4); i += 4) //Cycle over horizontal clusters
	{
		for (int j = 4; j < (img.cols - 4); j += 4) //Cycle over vertical clusters
		{
			xbar = 0;
			sse = 0;
			stddev = 0;
			op = 0;
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					xbar += img.at<Vec3b>(i + k, j + l)[0];
				}
			}
			xbar /= 16;
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					sse += pow(img.at<Vec3b>(i + k, j + l)[0] - xbar, 2);
				}
			}
			stddev = pow(sse / 16, 0.5);				
			if (imgFilter.at<uchar>(i, j) == 255 && (int)stddev >= deviation) op = 255;
			for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
			{
				for (int l = 0; l < 4; l++) //Cycle vertically within cluster
				{
					imgFilter.at<uchar>(i + k, j + l) = op;
				}
			}
		}
	}
}

/***********************************************************************
 * Step Four: Geometric Correction
 *
 * Further mophology:
 * 1. Pixel clusters with value 1 will remain one if surrounded by at
 *    least four clusters of value 1 in its local 3x3 neighborhood
 * 2. Pixel clusters with value 0 will be given value 1 if surrounded
 *    by at least 6 clusters of value 1 in its local 3x3 neighborhood
 *
 * Filter horizontally then vertically setting any runs of less than
 * four clusters to zero
 **********************************************************************/

void geometric_correction()
{
	int erode, dilate;
	for (int i = 4; i < (img.rows - 4); i+=4) //Cycle over horizontal clusters
	{
		for (int j = 4; j < (img.cols - 4); j+=4) //Cycle over vertical clusters
		{
			erode = 0;
			if (imgFilter.at<uchar>(i, j) == 255)
			{
				for (int k = -4; k < 5; k += 4)
				{
					for (int l = -4; l < 5; l += 4)
					{
						if (imgFilter.at<uchar>(i + k, j + l) == 255) erode++;
					}
				}
				if (erode < erode2)
				{
					for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
					{
						for (int l = 0; l < 4; l++) //Cycle vertically within cluster
						{
							imgFilter.at<uchar>(i + k, j + l) = 0;
						}
					}
				}
			}
		}
	}
	for (int i = 4; i < (img.rows - 4); i+=4) //Cycle over horizontal clusters
	{
		for (int j = 4; j < (img.cols - 4); j+=4) //Cycle over vertical clusters
		{
			dilate = 0;
			if (imgFilter.at<uchar>(i, j) < 255)
			{
				for (int k = -4; k < 5; k += 4)
				{
					for (int l = -4; l < 5; l += 4)
					{
						if (imgFilter.at<uchar>(i + k, j + l) == 255) dilate++;
					}
				}
				if (dilate > dilate2)
				{
					for (int k = 0; k < 4; k++) //Cycle horizontally within cluster
					{
						for (int l = 0; l < 4; l++) //Cycle vertically within cluster
						{
							imgFilter.at<uchar>(i + k, j + l) = 255;
						}
					}
				}
			}
		}
	}
}

void average_frame()
{
	accumulateWeighted(imgFilter, imgHist, ((double) weighting/255));
}
