#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QtSerialPort>
#include <QTimer>
#include <QString>
#include <QtSerialPort/QSerialPortInfo>
#include <QtDebug>
#include <QByteArray>
#include <QTextStream>
#include "qcustomplot.h"
#include <iostream>
#include <string>

#define RPMmax 360
#define RPMmin -360
#define smoothSteps 50

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
struct pid_Settings{
    float Kp, Ki, Kd;
} PID;

struct gamma_Settings{
    float gamma1, gamma2, gamma3;
} gamma;

union ByteToFloat{
    float myfloat;
    char mybyte[4];
} speed_data, send_data, position_data;
QString dir = "E:/BKel/201/DLCN/BTL2/driver_H_bridge/PID_Motor";
QDateTime datetime = QDateTime::currentDateTime();
private slots:
    void fillPortAction();
    void on_actionExit_triggered();
    void on_btn_setUart_clicked();
    void on_btn_closeUart_clicked();
    void Setup_Graph();
    void RealTimeData();
    void readData();
    void smooth_Setpoint();
    //void on_btn_clear_clicked();
    void on_btn_clear_clicked(bool checked);
    void on_btn_setPoint_clicked();
    void on_btn_setPID_clicked();
    void on_btn_setgamma_clicked();
    void on_btn_setZero_clicked();
    void on_btn_mode_clicked();
    void on_btn_setPosition_clicked();
    void stop_motor();
    void setPID();
    void setgamma();
    void displayText(QTextEdit* box, const QString &text, int fontsize);
    void displayText(QPushButton* box, const QString &text, int fontsize);
    void displayData();
    void readPIDgammadata();
    void writePIDgammadata();
    void writeData();
    void reference_model();
private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
