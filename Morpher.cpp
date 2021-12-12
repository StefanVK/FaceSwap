#include "Morpher.h"

#include "morph.h"

Morpher::Morpher(QObject* parent) : QObject(parent)
{
  moveToThread(&m_Thread);
  m_Thread.start();
}

Morpher::~Morpher()
{
  m_Thread.quit();
  m_Thread.wait();
}

void Morpher::startAsyncMorph(const MorphPrameters& morphParameters)
{ 
  QMetaObject::invokeMethod(this, "morph", Qt::QueuedConnection, Q_ARG(MorphPrameters, morphParameters));
}


void Morpher::morph(const MorphPrameters& morphParameters)
{
  m_isWorking = true;
  cv::Mat morphedImage = ::morph(morphParameters);
  MorphResult result;
  result.morphedImage = morphedImage;
  emit(morphed(result));
  m_isWorking = false;
}

