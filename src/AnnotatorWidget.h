#pragma once

#include "ImageWidget.h"

#include <QGraphicsView>

#include <opencv2/imgproc.hpp>

#include <filesystem>

class PointHandle;

class AnnotatorWidget : public ImageWidget
{
  Q_OBJECT;

public:
  AnnotatorWidget(QWidget* parent = nullptr);
  ~AnnotatorWidget();

  void mouseDoubleClickEvent(QMouseEvent* event) override;

  void keyPressEvent(QKeyEvent* event) override;

  void setZoom(double zoom);

  [[nodiscard]] std::vector<cv::Point2f> getPoints() const;
signals:
  void zoomChanged(double zoom);
  void pointCreated(const QPointF& pt, int ID);
  void pointMoved(const QPointF& pt, int ID);
  void pointsSelected(const std::vector<int>& ID);
  void pointRemoved(int ID);
  void changed();
public slots:
  int createPoint(const QPointF& pt, int ID);
  void highlighPoints(const std::vector<int>& ID);
  void removePoint(int ID);
  void clearDots();

private:
  double m_Zoom = 1.;
  double m_Radius = 7.;
  bool m_Pressed = false, m_Dragging = false;
  QString m_ImageFilename;
  std::map<int, PointHandle*> m_Circles;
};
