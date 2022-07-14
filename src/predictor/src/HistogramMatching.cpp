// HistogramMatching.cpp

//Include class
#include "HistogramMatching.h"

//Standard Libraries
#include <numeric>

HistogramMatching::HistogramMatching()
{ 
}

HistogramMatching::~HistogramMatching()
{
}

cv::Mat HistogramMatching::calculateLookup(const cv::Mat& src_cdf, const cv::Mat& ref_cdf)
{
	cv::Mat lookup_table(histSize, 1, CV_8U);
	for (int i = 0; i < histSize; i++)
	{
		for (int j = 0; j < histSize; j++)
		{
			if (ref_cdf.at<float>(j, 0) >= src_cdf.at<float>(i, 0))
			{
				//Since a 1280x960 pixel image, with 0-255 pixel intensity values, can be stored as an
				// 8-bit unsigned integer, so cv::Mat<uchar> can be used
				lookup_table.at<uchar>(i, 0) = j;
				break;
			}
		}

	}
	return lookup_table;
}

void HistogramMatching::normalizeMat(cv::Mat& mat)
{
	if (!mat.empty())
	{
		float last_val_inv = 1 / mat.at<float>(histSize - 1, 0); //Last value is the greatest in cumulative sum
		for (int i = 0; i < histSize; i++)
		{
			mat.at<float>(i, 0) = mat.at<float>(i, 0) * last_val_inv;
		}
	}
	else
		std::cout << "Array to be normalized is empty" << std::endl;
}

void HistogramMatching::cumulativeSum(const cv::Mat& input, cv::Mat& result)
{
	result.at<float>(0, 0) = input.at<float>(0, 0);
	for (int i = 1; i < histSize; i++)
	{
		result.at<float>(i, 0) = input.at<float>(i, 0)
			+ result.at<float>(i - 1, 0);
	}
	HistogramMatching::normalizeMat(result);
}

void HistogramMatching::histMatch(cv::Mat& img1, const cv::Mat& img2)
{

	//img1: Base image to be converted 
	//img2: Reference image to convert the base image
	cv::Mat img1_bgr[3], img2_bgr[3];
	//Split the multi-color channels to b, g, and r channels 
	cv::split(img1, img1_bgr);
	cv::split(img2, img2_bgr);
	float range[] = { 0, 256 };
	const float* histRange = { range };

	//Obtaining the histograms of B, G, and R channels in both the images
	cv::Mat hist1_b(histSize, 1, CV_32F), hist1_g(histSize, 1, CV_32F), hist1_r(histSize, 1, CV_32F),
		hist2_b(histSize, 1, CV_32F), hist2_g(histSize, 1, CV_32F), hist2_r(histSize, 1, CV_32F),
		hist1b_cumsum(histSize, 1, CV_32F), hist1g_cumsum(histSize, 1, CV_32F),
		hist1r_cumsum(histSize, 1, CV_32F), hist2b_cumsum(histSize, 1, CV_32F),
		hist2g_cumsum(histSize, 1, CV_32F), hist2r_cumsum(histSize, 1, CV_32F);

	//cv::calcHist(&img1_bgr[0], 1, 0, cv::Mat(), hist1b, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img1_bgr[0], 1, 0, cv::Mat(), hist1_b, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img1_bgr[1], 1, 0, cv::Mat(), hist1_g, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img1_bgr[2], 1, 0, cv::Mat(), hist1_r, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img2_bgr[0], 1, 0, cv::Mat(), hist2_b, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img2_bgr[1], 1, 0, cv::Mat(), hist2_g, 1, &histSize, &histRange, true, false);
	cv::calcHist(&img2_bgr[2], 1, 0, cv::Mat(), hist2_r, 1, &histSize, &histRange, true, false);

	//Compute the  cumulative sum
	HistogramMatching::cumulativeSum(hist1_b, hist1b_cumsum);
	HistogramMatching::cumulativeSum(hist1_g, hist1g_cumsum);
	HistogramMatching::cumulativeSum(hist1_r, hist1r_cumsum);
	HistogramMatching::cumulativeSum(hist2_b, hist2b_cumsum);
	HistogramMatching::cumulativeSum(hist2_g, hist2g_cumsum);
	HistogramMatching::cumulativeSum(hist2_r, hist2r_cumsum);

	//Lookup table implementation for storing data
	cv::Mat lookup_table_b = HistogramMatching::calculateLookup(hist1b_cumsum, hist2b_cumsum);
	cv::Mat lookup_table_g = HistogramMatching::calculateLookup(hist1g_cumsum, hist2g_cumsum);
	cv::Mat lookup_table_r = HistogramMatching::calculateLookup(hist1r_cumsum, hist2r_cumsum);

	//Transform img1 using the lookup table
	cv::Mat after_transform[3];
	cv::LUT(img1_bgr[0], lookup_table_b, after_transform[0]);
	cv::LUT(img1_bgr[1], lookup_table_g, after_transform[1]);
	cv::LUT(img1_bgr[2], lookup_table_r, after_transform[2]);

	std::vector<cv::Mat> channels = { after_transform[0], after_transform[1], after_transform[2] };
	//std::vector<float> t_vec;
	//Merge the channels
	cv::merge(channels, img1);
}
