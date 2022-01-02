#include "ImageWidget.h"

#include <QDockWidget>
#include <QFileDialog>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QLayout>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QTimer>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

ImageWidget::ImageWidget(QWidget* parent) : QGraphicsView(parent)
{
  QGraphicsScene* scene = new QGraphicsScene(this);
  setScene(scene);
  setDragMode(QGraphicsView::DragMode::ScrollHandDrag);
  setResizeAnchor(QGraphicsView::ViewportAnchor::AnchorUnderMouse);
  setTransformationAnchor(QGraphicsView::ViewportAnchor::AnchorUnderMouse);
  setMouseTracking(true);
}

void ImageWidget::setImage(const cv::Mat& image)
{
  if (image.empty())
    return;
  m_Mat = image.clone();
  m_Image = QImage(m_Mat.data, m_Mat.cols, m_Mat.rows, m_Mat.step, QImage::Format::Format_BGR888);
  m_Image.detach();
  setSceneRect(0, 0, m_Mat.cols, m_Mat.rows);
  scene()->update(sceneRect());
}

void ImageWidget::drawBackground(QPainter* painter, const QRectF& rect) { painter->drawImage(0, 0, m_Image); }

void ImageWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QMenu menu(this);
  QAction* saveAction = new QAction("Save Image", &menu);
  if (!m_Image.size().isEmpty())
    menu.addAction(saveAction);
  connect(saveAction, &QAction::triggered, [&]() {
    const QString filename =
        QFileDialog::getSaveFileName(this, "Save image as", QString(), "TIF (*.tif);; All files (*.*)");
    if (!filename.isEmpty())
    {
      if (!m_Image.save(filename))
        QMessageBox::critical(this, "Could not save image", "Saving image \"" + filename + "\" failed");
    }
  });
  menu.exec(event->globalPos());
}

void ImageWidget::wheelEvent(QWheelEvent* event)
{
  if (event->angleDelta().y() > 0 && m_Zoom < 10000)
  {
    setZoom(m_Zoom * std::pow(2, .2));
  }
  else if (m_Zoom > 0.0001)
    setZoom(m_Zoom * std::pow(2, -.2));
}

void ImageWidget::setZoom(double zoom)
{
  if (zoom == m_Zoom)
    return;

  scale(zoom / m_Zoom, zoom / m_Zoom);
  m_Zoom = zoom;
  emit(zoomChanged(zoom));
}
