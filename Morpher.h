#pragma once

#include <QObject>
#include <QThread>

#include <opencv2/core.hpp>

struct MorphPrameters;
struct MorphResult
{
  cv::Mat morphedImage;
};

class Morpher : public QObject
{
  Q_OBJECT;
public:
  Morpher(QObject* parent = nullptr);
  ~Morpher();
  [[nodiscard]] bool isWorking() const { return m_isWorking; }
  void startAsyncMorph(const MorphPrameters& morphParameters);
public slots:
  void morph(const MorphPrameters& morphParameters);
signals:
  void morphed(MorphResult);
private:
  QThread m_Thread;
  std::atomic<bool> m_isWorking{false};
};