#include "MainWindow.h"

#include "ui_MainWindow.h"
#include "morph.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>

#include <nlohmann/json.hpp>

#include <opencv2/imgcodecs.hpp>

#include <fstream>

MainWindow::MainWindow() : ui{std::make_unique<Ui::MainWindow>()}
{
  qRegisterMetaType<MorphPrameters>("MorphPrameters");
  
  ui->setupUi(this);
  connect(ui->actionLoad_project, &QAction::triggered, this, &MainWindow::LoadProject);
  connect(ui->actionSave_project, &QAction::triggered, this, &MainWindow::SaveProject);
  connect(ui->actionLoadImage_1, &QAction::triggered, this, [this]() { LoadImageDialog(0); });
  connect(ui->actionLoadImage_2, &QAction::triggered, this, [this]() { LoadImageDialog(1); });

  auto w1 = ui->graphicsView_1;
  auto w2 = ui->graphicsView_2;

  QObject::connect(w1, &AnnotatorWidget::pointCreated, this,
                   [&](const QPointF& pt, int ID) { CreateMatchingPoint(pt, ID, 0); });
  QObject::connect(w2, &AnnotatorWidget::pointCreated, this,
                   [&](const QPointF& pt, int ID) { CreateMatchingPoint(pt, ID, 1); });
  QObject::connect(w1, &AnnotatorWidget::pointsSelected, w2, &AnnotatorWidget::highlighPoints);
  QObject::connect(w2, &AnnotatorWidget::pointsSelected, w1, &AnnotatorWidget::highlighPoints);
  QObject::connect(w1, &AnnotatorWidget::pointRemoved, w2, &AnnotatorWidget::removePoint);
  QObject::connect(w2, &AnnotatorWidget::pointRemoved, w1, &AnnotatorWidget::removePoint);
  QObject::connect(w1, &AnnotatorWidget::changed, this, &MainWindow::MorphPrametersChanged);
  QObject::connect(w2, &AnnotatorWidget::changed, this, &MainWindow::MorphPrametersChanged);
  QObject::connect(ui->textureAlphaSlider, &QSlider::valueChanged, this, &MainWindow::MorphPrametersChanged);
  QObject::connect(ui->shapeAlphaSlider, &QSlider::valueChanged, this, &MainWindow::MorphPrametersChanged);
  QObject::connect(ui->smoothTransitionSlider, &QSlider::valueChanged, this, &MainWindow::MorphPrametersChanged);
  QObject::connect(ui->colorMatchCheckBox, &QCheckBox::stateChanged, this, &MainWindow::MorphPrametersChanged);

  QObject::connect(&m_Morpher, &Morpher::morphed, this, &MainWindow::SetMorphResult);
}

MainWindow::~MainWindow() {
}

namespace
{
std::vector<std::pair<float, float>> to_pairs(const std::vector<cv::Point2f>& points)
{
  std::vector<std::pair<float, float>> ret;
  std::ranges::transform(points, std::back_inserter(ret), [](const auto& p) { return std::make_pair(p.x, p.y); });
  return ret;
}
std::vector<cv::Point2f> to_cvPoints(const std::vector<std::pair<float, float>>& points)
{
  std::vector<cv::Point2f> ret;
  std::ranges::transform(points, std::back_inserter(ret), [](const auto& p) { return cv::Point2f{p.first, p.second}; });
  return ret;
}
} // namespace

void MainWindow::SaveProject()
{
  auto filename = QFileDialog::getSaveFileName(this, "Save FaceMorph project", "", "FaceMorph (*.json)");
  std::ofstream file(filename.toLocal8Bit());
  if (!file.is_open())
  {
    QMessageBox::critical(this, "Could not save project", "Could not open file");
    return;
  }
  nlohmann::json j;
  j["Files"] = m_Filenames;

  j["Dots"]["Img1"] = to_pairs(ui->graphicsView_1->getPoints());
  j["Dots"]["Img2"] = to_pairs(ui->graphicsView_2->getPoints());

  file << j;
}

void MainWindow::LoadImageDialog(int slot)
{
  auto filename =
      QFileDialog::getOpenFileName(this, "Load image", "", "JPEG (*.jpg *.jpeg);; TIFF (*.tif);; All files (*.*)");
  if (!filename.isEmpty())
  {
    m_Filenames[slot] = filename.toLocal8Bit();
    LoadImage(slot);
  }
}

void MainWindow::LoadImage(int slot)
{
  if (m_Filenames[slot].empty())
    return;
  cv::Mat img = cv::imread(m_Filenames[slot]);
  if (slot == 0)
  {
    ui->graphicsView_1->setImage(img);
  }
  if (slot == 1)
  {
    ui->graphicsView_2->setImage(img);
  }
  ui->graphicsView_1->clearDots();
  ui->graphicsView_2->clearDots();
}

void MainWindow::SetMorphResult(const MorphResult& morphResult)
{
  ui->morphedImage->setImage(morphResult.morphedImage);
  if (m_SceneChangedSinceLastMorph)
    StartAsyncMorph();
}

void MainWindow::MorphPrametersChanged()
{
  m_SceneChangedSinceLastMorph = true;
  if (!m_Morpher.isWorking())
  {
    StartAsyncMorph();
  }
}

void MainWindow::StartAsyncMorph() {
  m_SceneChangedSinceLastMorph = false;
  MorphPrameters params;
  params.img1 = ui->graphicsView_1->getImage();
  params.img2 = ui->graphicsView_2->getImage();
  if (params.img1.empty() || params.img2.empty())
    return;
  params.img1 = params.img1.clone();
  params.img2 = params.img2.clone();
  params.points1 = ui->graphicsView_1->getPoints();
  params.points2 = ui->graphicsView_2->getPoints();
  params.textureAlpha = ui->textureAlphaSlider->value() / 255.;
  params.shapeAlpha = ui->shapeAlphaSlider->value() / 255.;
  params.smoothTransitionAlpha = ui->smoothTransitionSlider->value() / 255.;
  params.colorMatch = ui->colorMatchCheckBox->isChecked();
  m_Morpher.startAsyncMorph(std::move(params));
}

void MainWindow::CreateMatchingPoint(const QPointF& point, const int ID, const int srcImg)
{
  const int dstImg = srcImg == 1 ? 0 : 1;
  std::array annotationWidgets{ui->graphicsView_1, ui->graphicsView_2};
  auto srcPoints = annotationWidgets[srcImg]->getPoints();
  auto dstPoints = annotationWidgets[dstImg]->getPoints();

  if (srcPoints.size() < 4 || dstPoints.size() < 4)
  {
    srcPoints.emplace_back(0, 0);
    srcPoints.emplace_back(annotationWidgets[srcImg]->getImage().cols, 0);
    srcPoints.emplace_back(annotationWidgets[srcImg]->getImage().cols, annotationWidgets[srcImg]->getImage().rows);
    srcPoints.emplace_back(0, annotationWidgets[srcImg]->getImage().rows);

    dstPoints.emplace_back(0, 0);
    dstPoints.emplace_back(annotationWidgets[dstImg]->getImage().cols, 0);
    dstPoints.emplace_back(annotationWidgets[dstImg]->getImage().cols, annotationWidgets[dstImg]->getImage().rows);
    dstPoints.emplace_back(0, annotationWidgets[dstImg]->getImage().rows);
  }
  cv::Point2f estimatedPoint = estimateNewPointCoordinate(
      cv::Point2f{static_cast<float>(point.x()), static_cast<float>(point.y())}, srcPoints, dstPoints);
  annotationWidgets[dstImg]->createPoint(QPointF{estimatedPoint.x, estimatedPoint.y}, ID);
}

void MainWindow::LoadProject()
{
  auto filename = QFileDialog::getOpenFileName(this, "Open FaceMorph project", "", "FaceMorph (*.json)");
  if (!filename.isEmpty())
  {
    m_Filenames = {};
    std::ifstream file(filename.toLocal8Bit());
    if (!file.is_open())
      return;
    nlohmann::json j = nlohmann::json::parse(file);
    auto files = j["Files"];
    if (files.size() > 0 && files[0].is_string())
      m_Filenames[0] = files[0];
    if (files.size() > 1 && files[1].is_string())
      m_Filenames[1] = files[1];
    LoadImage(0);
    LoadImage(1);

    auto points = to_cvPoints(j["Dots"]["Img1"]);
    for (int i = 0; i < points.size(); ++i)
      ui->graphicsView_1->createPoint({points[i].x, points[i].y}, i);

    points = to_cvPoints(j["Dots"]["Img2"]);
    for (int i = 0; i < points.size(); ++i)
      ui->graphicsView_2->createPoint({points[i].x, points[i].y}, i);
  }
}
