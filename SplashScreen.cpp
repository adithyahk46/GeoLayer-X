#include "SplashScreen.h"


SplashScreen::SplashScreen(QPixmap& pixmap)
{
  int imageSize = 350;
  int margin = 20;

  nowStep = 0;
  progressBar = new QProgressBar(this);
  progressBar->setGeometry(margin, imageSize - margin, imageSize - margin, margin);
  progressBar->setTextVisible(false);

  rePixmap = pixmap.scaledToHeight(imageSize, Qt::SmoothTransformation);
  this->setPixmap(rePixmap);

  splashFont.setFamily("Arial");
  splashFont.setBold(true);
  splashFont.setPixelSize(15);
  splashFont.setStretch(125);

  this->setFont(splashFont);

}

SplashScreen::~SplashScreen()
{
}

void SplashScreen::setTotalInitSteps(int num) {
  progressBar->setRange(0, num);
  totalSteps = num;
}

void SplashScreen::setNowInitName(const QString& name)
{
  nowStep++;
  progressBar->setValue(nowStep);
  this->showMessage(tr("%1\n").arg(name),
    Qt::AlignHCenter | Qt::AlignBottom, Qt::white);
}
