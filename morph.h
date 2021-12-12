#pragma once
#include <opencv2/core.hpp>

#include <vector>

struct MorphPrameters
{
  cv::Mat img1, img2;
  std::vector<cv::Point2f> points1, points2;
  double textureAlpha = .5, shapeAlpha = .5, smoothTransitionAlpha = .5;
  bool colorMatch = false;
};

cv::Mat morph(const MorphPrameters& params);


cv::Point2f estimateNewPointCoordinate(cv::Point2f point, std::vector<cv::Point2f> srcPoints,
                                       const std::vector<cv::Point2f>& dstPoints);