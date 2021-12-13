#include <QApplication>
#include <QTimer>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include "AnnotatorWidget.h"
#include "ImageWidget.h"
#include "MainWindow.h"
#include "morph.h"

namespace
{
// https://stackoverflow.com/questions/35698647/how-to-get-vertex-index-from-subdiv2d-delaunay-triangulation
class Subdiv2DIndex : public cv::Subdiv2D
{
public:
  explicit Subdiv2DIndex(const cv::Rect& rectangle) : cv::Subdiv2D{rectangle} {}

  // Source code of Subdiv2D:
  // https://github.com/opencv/opencv/blob/master/modules/imgproc/src/subdivision2d.cpp#L762 The implementation
  // tweaks getTrianglesList() so that only the indice of the triangle inside the image are returned
  void getTrianglesIndices(std::vector<int>& triangleList) const
  {
    triangleList.clear();
    int total = (int)(qedges.size() * 4);
    std::vector<bool> edgemask(total, false);
    cv::Rect2f rect(topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);

    for (int i = 4; i < total; i += 2)
    {
      if (edgemask[i])
        continue;
      cv::Point2f a, b, c;
      int edge_a = i;
      int indexA = edgeOrg(edge_a, &a) - 4;
      if (!rect.contains(a))
        continue;
      int edge_b = getEdge(edge_a, NEXT_AROUND_LEFT);
      int indexB = edgeOrg(edge_b, &b) - 4;
      if (!rect.contains(b))
        continue;
      int edge_c = getEdge(edge_b, NEXT_AROUND_LEFT);
      int indexC = edgeOrg(edge_c, &c) - 4;
      if (!rect.contains(c))
        continue;
      edgemask[edge_a] = true;
      edgemask[edge_b] = true;
      edgemask[edge_c] = true;

      triangleList.push_back(indexA);
      triangleList.push_back(indexB);
      triangleList.push_back(indexC);
    }
  }
};

// Apply affine transform calculated using srcTri and dstTri to src
void applyAffineTransform(cv::Mat& warpImage, const cv::Mat& src, const std::vector<cv::Point2f>& srcTri,
                          const std::vector<cv::Point2f>& dstTri)
{

  // Given a pair of triangles, find the affine transform.
  cv::Mat warpMat = cv::getAffineTransform(srcTri, dstTri);

  // Apply the Affine Transform just found to the src image
  cv::warpAffine(src, warpImage, warpMat, warpImage.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
}

// Warps and alpha blends triangular regions from img1 and img2 to img
void morphTriangle(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& dst, const std::vector<cv::Point2f>& t1,
                   const std::vector<cv::Point2f>& t2, const std::vector<cv::Point2f>& t, double alpha)
{
  if (t1.size() < 3 || t2.size() < 3 || t.size() < 3)
    return;
  // Find bounding rectangle for each triangle
  cv::Rect r = boundingRect(t);
  if (!cv::Rect(0, 0, dst.cols, dst.rows).contains(r.tl()) || !cv::Rect(0, 0, dst.cols, dst.rows).contains(r.br()))
    return;
  cv::Rect r1 = boundingRect(t1);
  if (!cv::Rect(0, 0, img1.cols, img1.rows).contains(r1.tl()) ||
      !cv::Rect(0, 0, img1.cols, img1.rows).contains(r1.br()))
    return;
  cv::Rect r2 = boundingRect(t2);
  if (!cv::Rect(0, 0, img2.cols, img2.rows).contains(r2.tl()) ||
      !cv::Rect(0, 0, img2.cols, img2.rows).contains(r2.br()))
    return;

  // Offset points by left top corner of the respective rectangles
  std::vector<cv::Point2f> t1Rect, t2Rect, tRect;
  std::vector<cv::Point> tRectInt;
  for (int i = 0; i < 3; i++)
  {
    tRect.push_back(cv::Point2f(t[i].x - r.x, t[i].y - r.y));
    tRectInt.push_back(cv::Point(t[i].x - r.x, t[i].y - r.y)); // for fillConvexPoly

    t1Rect.push_back(cv::Point2f(t1[i].x - r1.x, t1[i].y - r1.y));
    t2Rect.push_back(cv::Point2f(t2[i].x - r2.x, t2[i].y - r2.y));
  }

  // Get mask by filling triangle
  cv::Mat mask = cv::Mat::zeros(r.height, r.width, CV_32FC3);
  cv::fillConvexPoly(mask, tRectInt, cv::Scalar(1.0, 1.0, 1.0), 16, 0);

  // Apply warpImage to small rectangular patches
  cv::Mat img1Rect, img2Rect;
  img1(r1).copyTo(img1Rect);
  img2(r2).copyTo(img2Rect);

  cv::Mat warpImage1 = cv::Mat::zeros(r.height, r.width, img1Rect.type());
  cv::Mat warpImage2 = cv::Mat::zeros(r.height, r.width, img2Rect.type());

  applyAffineTransform(warpImage1, img1Rect, t1Rect, tRect);
  applyAffineTransform(warpImage2, img2Rect, t2Rect, tRect);

  // Alpha blend rectangular patches
  cv::Mat imgRect = (1.0 - alpha) * warpImage1 + alpha * warpImage2;

  // Copy triangular region of the rectangular patch to the output image
  multiply(imgRect, mask, imgRect, 1.0, CV_8UC3);
  multiply(dst(r), cv::Scalar(1.0, 1.0, 1.0) - mask, dst(r), 1.0, CV_8UC3);
  dst(r) = dst(r) + imgRect;
}

void morph(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& dst, std::vector<cv::Point2f>& t1,
           std::vector<cv::Point2f>& t2, std::vector<cv::Point2f>& t, double alpha)
{
  for (int i = 0; i + 2 < t1.size(); i += 3)
  {

    std::vector<cv::Point2f> tps1{t1[i], t1[i + 1], t1[i + 2]}, tps2{t2[i], t2[i + 1], t2[i + 2]},
        tps{t[i], t[i + 1], t[i + 2]};

    morphTriangle(img1, img2, dst, tps1, tps2, tps, alpha);
  }
}
} // namespace

// Draw delaunay triangles
static void draw_delaunay(cv::Mat& img, cv::Subdiv2D& subdiv)
{
  const cv::Scalar delaunay_color(255, 255, 255);
  std::vector<cv::Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  std::vector<cv::Point> pt(3);
  cv::Size size = img.size();
  cv::Rect rect(0, 0, size.width, size.height);

  for (size_t i = 0; i < triangleList.size(); i++)
  {
    cv::Vec6f t = triangleList[i];
    pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
    pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
    pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

    // Draw rectangles completely inside the image.
    if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
    {
      line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
      line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
      line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
    }
  }
}

static void draw_delaunay_filled(cv::Mat& img, cv::Subdiv2D& subdiv)
{
  const cv::Scalar delaunay_color(255, 255, 255);
  std::vector<cv::Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  std::vector<cv::Point> pt(3);
  cv::Size size = img.size();
  cv::Rect rect(0, 0, size.width, size.height);

  for (size_t i = 0; i < triangleList.size(); i++)
  {
    cv::Vec6f t = triangleList[i];
    pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
    pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
    pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

    // Draw rectangles completely inside the image.
    if (rect.contains(pt[0]) || rect.contains(pt[1]) || rect.contains(pt[2]))
    {
      cv::drawContours(img, std::vector<std::vector<cv::Point>>{{pt}}, 0, 255, -1);
    }
  }
}

cv::Point2f mean(const std::vector<cv::Point2f>& points)
{
  const cv::Point2f sum = std::accumulate(points.begin(), points.end(), cv::Point2f{});
  const cv::Point2f mean{sum.x / points.size(), sum.y / points.size()};
  return mean;
}

cv::Point2f abs(const cv::Point2f& p) { return cv::Point2f{std::abs(p.x), std::abs(p.y)}; }

cv::Point2f averageDistance(const std::vector<cv::Point2f>& points, const cv::Point2f& center)
{
  const cv::Point2f sum =
      std::accumulate(points.begin(), points.end(), cv::Point2f{},
                      [&center](const cv::Point2f& sum, const cv::Point2f& cur) { return sum + abs(cur - center); });
  const cv::Point2f average{sum.x / points.size(), sum.y / points.size()};
  return average;
}

std::vector<cv::Point2f> morphPoints(const std::vector<cv::Point2f>& srcPoints,
                                     const std::vector<cv::Point2f>& dstPoints, double shapeAlpha)
{
  // compute weighted average point coordinates
  std::vector<cv::Point2f> morphedPoints;
  for (int i = 0; i < srcPoints.size(); i++)
  {
    float x, y;
    x = (1 - shapeAlpha) * srcPoints[i].x + shapeAlpha * dstPoints[i].x;
    y = (1 - shapeAlpha) * srcPoints[i].y + shapeAlpha * dstPoints[i].y;
    morphedPoints.push_back(cv::Point2f(x, y));
  }
  cv::Point2f dstCenter = mean(dstPoints);
  cv::Point2f dstDist = averageDistance(dstPoints, dstCenter);
  cv::Point2f morphedCenter = mean(morphedPoints);
  cv::Point2f morphedDist = averageDistance(morphedPoints, morphedCenter);
  for (auto& p : morphedPoints)
  {
    cv::Point2f scale{dstDist.x / morphedDist.x, dstDist.y / morphedDist.y};
    p = dstCenter + cv::Point2f{(p - morphedCenter).x * scale.x, (p - morphedCenter).y * scale.y};
  }
  return morphedPoints;
}

std::array<std::array<int, 256>, 3> calcHist(const cv::Mat& mat, const cv::Mat& mask)
{
  assert(mat.size == mask.size);
  assert(mat.type() == CV_8UC3);
  assert(mask.type() == CV_8UC1);

  std::array<std::array<int, 256>, 3> hist{};
  for (int y = 0; y < mat.rows; ++y)
    for (int x = 0; x < mat.cols; ++x)
    {
      if (mask.at<uint8_t>(cv::Point(x, y)))
      {
        const cv::Vec3b& color = mat.at<cv::Vec3b>(cv::Point(x, y));
        for (int channel = 0; channel < 3; ++channel)
          ++hist[channel][color[channel]];
      }
    }
  return hist;
}

std::array<std::array<int, 256>, 3> calcHistoMatchLUT(const std::array<std::array<int, 256>, 3>& srcHist,
                                                      const std::array<std::array<int, 256>, 3>& refHist)
{
  std::array<std::array<int, 256>, 3> LUT{};
  for (int channel = 0; channel < 3; ++channel)
  {
    int64_t srcSum = 0, refSum = 0;
    for (int srcGV = 0, refGV = 0; srcGV <= 255; ++srcGV)
    {
      while (srcSum > refSum)
      {
        refSum += refHist[channel][refGV];
        ++refGV;
      }

      srcSum += srcHist[channel][srcGV];
      LUT[channel][srcGV] = refGV;
    }
  }
  return LUT;
}

cv::Mat histoMatch(const cv::Mat& src, const cv::Mat& ref, const cv::Mat& mask)
{
  auto srcHist = calcHist(src, mask);
  auto refHist = calcHist(ref, mask);
  auto LUT = calcHistoMatchLUT(srcHist, refHist);
  cv::Mat matched = src.clone();
  for (int y = 0; y < src.rows; ++y)
    for (int x = 0; x < src.cols; ++x)
    {
      if (mask.at<uint8_t>(cv::Point(x, y)))
      {
        const cv::Vec3b& srcColor = src.at<cv::Vec3b>(cv::Point(x, y));
        cv::Vec3b& matchedColor = matched.at<cv::Vec3b>(cv::Point(x, y));
        for (int channel = 0; channel < 3; ++channel)
          matchedColor[channel] = LUT[channel][srcColor[channel]];
      }
    }
  return matched;
}

cv::Mat morph(const MorphPrameters& params)
{

  if (params.points1.size() != params.points2.size())
    return {};
  if (params.points1.size() < 3)
    return {};

  std::vector<cv::Point2f> points = morphPoints(params.points1, params.points2, params.shapeAlpha);

  const cv::Rect br = cv::boundingRect(points);
  Subdiv2DIndex subdiv1(br);
  subdiv1.insert(points);

  std::vector<cv::Vec6f> triangles;
  subdiv1.getTriangleList(triangles);
  std::vector<int> indices;
  subdiv1.getTrianglesIndices(indices);

  std::vector<cv::Point2f> tps1, tps2, tps;

  for (int i = 0; i < indices.size(); ++i)
  {
    tps1.push_back(params.points1[indices[i]]);
    tps2.push_back(params.points2[indices[i]]);
    tps.push_back(points[indices[i]]);
  }
  cv::Mat dst = params.img2.clone();
  morph(params.img1, params.img2, dst, tps1, tps2, tps, params.textureAlpha);

  cv::Mat seamDst = params.img2.clone();
  cv::Mat mask(dst.size(), CV_8UC1);
  mask = 0;
  draw_delaunay_filled(mask, subdiv1);

  if (params.colorMatch)
    dst = ::histoMatch(dst, params.img2, mask);

  cv::Rect roi_s = boundingRect(mask);
  auto p = (roi_s.tl() + roi_s.br()) / 2;

  cv::seamlessClone(dst, params.img2, mask, p, seamDst, cv::NORMAL_CLONE);
  cv::Mat ret = params.smoothTransitionAlpha * seamDst + (1 - params.smoothTransitionAlpha) * dst;
  return ret;
}



cv::Point2f estimateNewPointCoordinate(cv::Point2f point, std::vector<cv::Point2f> srcPoints,
                                       const std::vector<cv::Point2f>& dstPoints)
{
  if (srcPoints.size() < 4 || dstPoints.size() < 4)
    return point;
  if (srcPoints.size() > dstPoints.size())
    srcPoints.erase(std::ranges::find(srcPoints, point));
  
  cv::Point2f srcMean = mean(srcPoints);
  cv::Point2f srcDist = averageDistance(srcPoints, srcMean);
  cv::Point2f dstMean = mean(dstPoints);
  cv::Point2f dstDist = averageDistance(dstPoints, dstMean);

  cv::Point2f scale{dstDist.x / srcDist.x, dstDist.y / srcDist.y};
  cv::Point2f ret = dstMean + cv::Point2f{(point - srcMean).x * scale.x, (point - srcMean).y * scale.y};
  return ret;
}