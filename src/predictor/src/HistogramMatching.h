// HistogramMatching.h
#pragma once
#ifndef HistogramMatching_H
#define HistogramMatching_H

//OpenCV Libraries
#include <opencv2/opencv.hpp>

class HistogramMatching
{
public:
	const int histSize = 256; //An image will always have 256 bins of intensity values [0-255]

	HistogramMatching(); //Default constructor

	~HistogramMatching(); //Default destructor

	cv::Mat calculateLookup(const cv::Mat& src_cdf, const cv::Mat& ref_cdf);

	void normalizeMat(cv::Mat& mat);

	void cumulativeSum(const cv::Mat& input, cv::Mat& result);

	void histMatch(cv::Mat& img1, const cv::Mat& img2);
};

#endif 
