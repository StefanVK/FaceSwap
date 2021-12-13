#pragma once

#include "ImageWidget.h"

#include <QGraphicsView>

#include <opencv2/imgproc.hpp>

#include <filesystem>

class AnnotatorWidget : public ImageWidget
{
  Q_OBJECT;

public:
  AnnotatorWidget(QWidget* parent = nullptr);
  ~AnnotatorWidget();

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
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
  std::map<int, QGraphicsEllipseItem*> m_Circles;
};