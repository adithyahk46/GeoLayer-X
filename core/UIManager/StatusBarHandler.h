#ifndef STATUSBARHANDLER_H
#define STATUSBARHANDLER_H


#include <QStatusBar>
#include <QObject>

class StatusBarHandler: public QObject
{
    Q_OBJECT

public:
    static StatusBarHandler* getInstance();

    void setStatusbar(QStatusBar* StatusBar){_sbWidget = StatusBar;}

    void CreatMapReaders();

private:
    StatusBarHandler();

    static StatusBarHandler *_statusbar;
    QStatusBar* _sbWidget = nullptr;


    QLineEdit* _lat;
    QLineEdit* _long;
    QLineEdit* _height;
    QComboBox* _CRS;
    QLabel* _label;

};

#endif // STATUSBARHANDLER_H
