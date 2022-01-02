#pragma once

#include <QGraphicsView>

#include <opencv2/core.hpp>

class ImageWidget : public QGraphicsView
{
  Q_OBJECT;

public:
  ImageWidget(QWidget* parent = nullptr);
  void setImage(const cv::Mat& image);
  const cv::Mat& getImage() const { return m_Mat; }
  double getZoom() const { return m_Zoom; }
signals:
  void zoomChanged(double zoom);
public slots:
  void setZoom(double zoom);
protected:
  void drawBackground(QPainter* painter, const QRectF& rect) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
private:
  double m_Zoom = 1.0;
protected:
  cv::Mat m_Mat;
  QImage m_Image;
};
