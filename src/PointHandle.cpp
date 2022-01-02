#include "PointHandle.h"

#include <QPainter>
namespace
{
QTransform GenerateTranslationOnlyTransform(const QTransform& original_transform, const QPointF& target_point)
{
  // https://www.generacodice.com/en/articolo/4511711/maintaining-relative-child-position-after-applying-qgraphicsitem-itemignorestransformations
  // To draw the unscaled icons, we desire a transform with scaling factors
  // of 1 and shearing factors of 0 and the appropriate translation such that
  // our icon center ends up at the same point. According to the
  // documentation, QTransform transforms a point in the plane to another
  // point using the following formulas:
  // x' = m11*x + m21*y + dx
  // y' = m22*y + m12*x + dy
  //
  // For our new transform, m11 and m22 (scaling) are 1, and m21 and m12
  // (shearing) are 0. Since we want x' and y' to be the same, we have the
  // following equations:
  // m11*x + m21*y + dx = x + dx[new]
  // m22*y + m12*x + dy = y + dy[new]
  //
  // Thus,
  // dx[new] = m11*x - x + m21*y + dx
  // dy[new] = m22*y - y + m12*x + dy
  const qreal dx = original_transform.m11() * target_point.x() - target_point.x() +
                   original_transform.m21() * target_point.y() + original_transform.m31();
  const qreal dy = original_transform.m22() * target_point.y() - target_point.y() +
                   original_transform.m12() * target_point.x() + original_transform.m32();

  return QTransform::fromTranslate(dx, dy);
}
} // namespace

PointHandle::PointHandle(const QPointF& pt)
    : QGraphicsEllipseItem(pt.x() - m_Radius, pt.y() - m_Radius, m_Radius * 2, m_Radius * 2)
{
}

// Overriden to paint the points at the same size regardless of scene scale
void PointHandle::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  // https://stackoverflow.com/questions/1222914/qgraphicsview-and-qgraphicsitem-don%C2%B4t-scale-item-when-scaling-the-view-rect
  const QTransform t = painter->transform();
  const qreal m11 = t.m11(), m22 = t.m22();
  painter->save(); 
  painter->setTransform(GenerateTranslationOnlyTransform(t, rect().center()));
  QGraphicsEllipseItem::paint(painter, option, widget);
  painter->restore();
}
