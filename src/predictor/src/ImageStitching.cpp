// ImageStitching.cpp

//Include class
#include "ImageStitching.h"
#include "HistogramMatching.h"

using namespace std;
using namespace cv;

ImageStitching::ImageStitching()
{
}

ImageStitching::~ImageStitching()
{
}


//'alignImages' function called inside 'homCalc' function im1 is "delayed" Omnicam image im2 is "delayed" PTZ image
void ImageStitching::alignImages(Mat& im1, Mat& im2, Mat& h2, Mat& h1, Parameters* P)
{   
    if (!im1.empty() && !im2.empty())
    { 
        //Mat h21 = h1 * (*P).H; //Predicted PTZ image in the Omnicam frame ??OLD
        Mat h21 = h1 * h2 * (*P).H; //Predicted PTZ image in the Omnicam frame
        
        // Use homography to warp image
    	//Warp delayed Omni image to delayed PTZ image in Omni frame 
    	Mat im1delayed;
    	warpPerspective(im1, im1delayed, h2*(*P).H, im2.size());
    	
    	//Warp delayed PTZ image in Omni frame to predicted PTZ image in Omni frame 
    	Mat im1pred;
    	warpPerspective(im1delayed, im1pred, h1, im2.size());
    	   	
    	Mat im21Omni, im21PTZ;
    	
    	//Warp the delayed PTZ image to predicted PTZ image
    	warpPerspective(im2, im21PTZ, h1, im2.size());
    	
    	//Warp the delayed omni image to predicted PTZ image in omni frame
    	warpPerspective(im1, im21Omni, h21, im2.size());
    	
    	//std::cout << (*P).stitch << " " << (*P).PD << std::endl; 
    	//HistogramMatching HM2; //Creating an object of the 'HistogramMatching' class
	if ((*P).stitch == 0)
    	{
    	    //HM2.histMatch(im1delayed, im2); 
    	    namedWindow("PTZ", WND_PROP_FULLSCREEN);
            setWindowProperty("PTZ", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN); 
            circle(im2, Point(960, 540), 5, Scalar(0, 0, 255), FILLED);
            imshow("PTZ", im2);
            
    	    namedWindow("Omni", WND_PROP_AUTOSIZE);
    	    moveWindow("Omni", 0, 0);
            setWindowProperty("Omni", WND_PROP_AUTOSIZE, WINDOW_GUI_NORMAL); 
            circle(im21Omni, Point(960, 540), 5, Scalar(0, 0, 255), FILLED);
    	    cv::resize(im21Omni, im21Omni, cv::Size(640, 360));
    	    
    	    imshow("Omni", im21Omni);
    	    waitKey(1);    	       	    
    	}
    	else
    	{
    	    Mat dst_subtract, dst_subtract_omni;
    	    subtract(im21Omni, im1pred, dst_subtract);
    	
    	    Mat ouImage;
    	    add(im21PTZ, dst_subtract, ouImage);
    	    namedWindow("Final", WND_PROP_FULLSCREEN);
            setWindowProperty("Final", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN); 
            circle(ouImage, Point(960, 540), 5, Scalar(0, 0, 255), FILLED);
    	    
            imshow("Final", ouImage);
            waitKey(1); 
            
            //For estimating the algorithm time 
            //(*P).end_alg = ros::Time::now().toSec();
            //std::cout << ((*P).end_alg - (*P).begin_alg)*1000 << std::endl; 
            //(*P).begin_alg = (*P).end_alg;	
    	}
    }
}

void ImageStitching::imageCb_omni(const sensor_msgs::ImageConstPtr& msg, Parameters* P)
{
	//cout << "In image_cb_omni: " << (*P).pi << endl;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//CV_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat src_omni = cv_ptr->image;

	// Crop image
	cv::Rect myROI = cv::Rect(1504, 0, 1504, 1504);
	Mat orig(src_omni, myROI);

	// Resize image
	resize(orig, orig, Size(1440, 1440), 0);
        
        
        cv::Mat omni_und;
	// Undistort image
	fisheye::undistortImage(orig, omni_und, (*P).Ko, (*P).Do, (*P).Ko, Size(orig.cols, orig.rows));

	// Flip image
	flip(omni_und, (*P).orig_show_omni, -1);
	
	//imshow("image_omni", (*P).orig_show_omni);
    	//cv::waitKey(1);
}

void ImageStitching::imageCb_ptz(const sensor_msgs::ImageConstPtr& msg, Parameters& P)
{
    cv_bridge::CvImagePtr cv_ptr;
    //Get time header info
    std_msgs::Header h = msg->header;
    //P.tau0 = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
    //double tau1 = ros::Time::now().toSec();//chrono::system_clock::to_time_t(chrono::system_clock::now());
    //double tau = tau1 - P.tau0;
    //cout << "Total delay: " << setprecision(15) << tau*1000 << " ms" << endl;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //CV_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat src_ptz;
    src_ptz = cv_ptr->image;

    // To show image
    //Mat orig_show;
    undistort(src_ptz, P.orig_show_PTZ, P.Kp, P.Dp, P.Kp_new);
    //resize(src_ptz, P.orig_show_PTZ, src_ptz.size(), 0);

    //imshow("image_ptz", orig_show_PTZ);
    //cv::waitKey(1);
}
