#pragma once

#include <QMainWindow>
#include <QTimer>

#include "Morpher.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT;
public:
  MainWindow();
  ~MainWindow();

private slots:
  void LoadProject();
  void SaveProject();
  void LoadImageDialog(int slot);
  void LoadImage(int slot);
  void SetMorphResult(const MorphResult& morphResult);
  void MorphPrametersChanged();
  void StartAsyncMorph();
  void CreateMatchingPoint(const QPointF& point, int ID, int srcImg);

private:
  std::unique_ptr<Ui::MainWindow> ui;
  std::array<std::string, 2> m_Filenames;
  bool m_SceneChangedSinceLastMorph = false;

  Morpher m_Morpher;
};
