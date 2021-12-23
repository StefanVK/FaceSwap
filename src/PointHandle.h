#include <QGraphicsItem>

class PointHandle : public QGraphicsEllipseItem
{
public:
  PointHandle(const QPointF& pt);
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

private:
  static constexpr int m_Radius = 3;
};