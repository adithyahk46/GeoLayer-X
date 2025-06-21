#ifndef SPLASHSCREEN_H
#define SPLASHSCREEN_H

#include <QSplashScreen>
#include <QSplashScreen>
#include <QProgressBar>
#include <QPixmap>

class SplashScreen  : public QSplashScreen
{
    Q_OBJECT
public:
    explicit SplashScreen(QPixmap& pixmap);



    ~SplashScreen();
private:
    QFont splashFont;
    QPixmap rePixmap;
    QProgressBar *progressBar;
    int totalSteps, nowStep;

private slots:
    void setTotalInitSteps(int num);
    void setNowInitName(const QString& name);
};

#endif // SPLASHSCREEN_H
