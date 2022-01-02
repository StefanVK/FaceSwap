#include "AnnotatorWidget.h"

#include "PointHandle.h"

#include <QGraphicsItem>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>

#include <opencv2/imgcodecs.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>

AnnotatorWidget::AnnotatorWidget(QWidget* parent) : ImageWidget(parent)
{
  QGraphicsScene* gscene = new QGraphicsScene(this);
  connect(gscene, &QGraphicsScene::selectionChanged, this, [&]() {
    auto selected = scene()->selectedItems();
    std::vector<int> IDs;
    for (const auto& sel : selected)
    {
      const int ID = qvariant_cast<int>(sel->data(0));
      IDs.push_back(ID);
    }
    highlighPoints(IDs);
    emit(pointsSelected(IDs));
  });

  connect(gscene, &QGraphicsScene::changed, this, &AnnotatorWidget::changed);
  
  setScene(gscene);
  setMouseTracking(true);
}

AnnotatorWidget::~AnnotatorWidget() = default;

void AnnotatorWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton)
  {
    QPointF pt = mapToScene(event->pos());
    auto itemsAtPoint = scene()->items(pt);
    if (itemsAtPoint.size() < 2) // The image is one graphics item as well
    {
      const int ID = createPoint(pt, -1);
      emit(pointCreated(pt, ID));
    }
  }
}

void AnnotatorWidget::keyPressEvent(QKeyEvent* event)
{
  if (event->key() == Qt::Key_Delete)
  {
    auto selectedItems = scene()->selectedItems();
    for (auto& selecteditem : selectedItems)
    {
      const int ID = qvariant_cast<int>(selecteditem->data(0));
      removePoint(ID);
    }
  }

  if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Minus)
  {
    auto selectedItems = scene()->selectedItems();
    for (auto& selectedItem : selectedItems)
    {
      auto pcircle = dynamic_cast<QGraphicsEllipseItem*>(selectedItem);
      if (pcircle)
      {
        if (event->key() == Qt::Key_Plus)
          pcircle->setRect(pcircle->rect().adjusted(-.5, -.5, .5, .5));
        if (event->key() == Qt::Key_Minus)
          pcircle->setRect(pcircle->rect().adjusted(.5, .5, -.5, -.5));
        m_Radius = (pcircle->rect().width() + pcircle->rect().height()) / 4;
      }
    }
  }

  QGraphicsView::keyPressEvent(event);
}

void AnnotatorWidget::setZoom(double zoom)
{
  if (zoom == m_Zoom)
    return;

  scale(zoom / m_Zoom, zoom / m_Zoom);
  m_Zoom = zoom;
  emit(zoomChanged(zoom));
}

std::vector<cv::Point2f> AnnotatorWidget::getPoints() const
{
  std::vector<cv::Point2f> points;
  for ([[maybe_unused]] auto& [ID, circle] : m_Circles)
  {
    auto movedRect = circle->rect();
    movedRect.translate(circle->scenePos()); 
    const QPointF pos = movedRect.center();
    points.emplace_back(pos.x(), pos.y());
  }
  return points;
}

void AnnotatorWidget::highlighPoints(const std::vector<int>& IDs)
{
  for ([[maybe_unused]] auto& [ID, item] : m_Circles)
  {
    item->setBrush(QBrush{QColor{0, 255, 0, 120}});
    item->setPen(QPen{QColor(0, 128, 128, 70)});
  }
  for (auto ID : IDs)
  {
    if (m_Circles.contains(ID))
    {
      m_Circles[ID]->setBrush(QBrush{QColor{255, 0, 0, 200}});
      m_Circles[ID]->setPen(QPen{QColor(255, 70, 70, 70)});
    }
  }
}

void AnnotatorWidget::removePoint(int ID)
{
  if (m_Circles.contains(ID))
  {
    delete m_Circles[ID];
    m_Circles.erase(ID);
    emit(pointRemoved(ID));
    highlighPoints({});
  }
}

void AnnotatorWidget::clearDots()
{
  for ([[maybe_unused]] auto& [ID, circle] : m_Circles)
    delete circle;
  m_Circles.clear();
  emit(changed());
}

int AnnotatorWidget::createPoint(const QPointF& pt, int ID)
{
  if (ID == -1)
    while (m_Circles.contains(++ID))
      ;
  const int radius = 3;
  auto circle = new PointHandle(pt);
  scene()->addItem(circle);
  circle->setFlag(QGraphicsItem::ItemIsMovable);
  circle->setFlag(QGraphicsItem::ItemIsSelectable);
  circle->setData(0, ID);
  const bool newPoint = !m_Circles.contains(ID);
  if (!newPoint)
    delete m_Circles[ID];
  m_Circles[ID] = (circle);
  highlighPoints({ID});
  return ID;
}
