#include "mainwindow.h"
#include "ui_mainwindow.h"
/*
 * Toi 17/12/2020
 * 1. Cap nhat PID khi khoi dong
 * 2. Viet ham update so trong box
 * 3. Update so moi lan chay
 * 4. Doc, xuat file
 * 5. Doc, xuat gia tri PID
 * 6. Xuat du lieu ra file (chua test)
 */
QSerialPort *SerialPort;
QTimer *timer, *timer_Uart, *timer_Smoothing;
float setpoint = 0;
float resolution = 0;
float pre_setpoint = 0;
float rt = 0;
float rt_1 = 0;
float rt_2 = 0;
float ym = 0;
float ym_1 = 0;
float ym_2 = 0;
int count = 0;
bool flag_PlotTimer;
struct MainWindow::pid_Settings PID = {0, 0, 0};
struct MainWindow::gamma_Settings gamma = {0, 0, 0};
int mode = 0; //mode 0: speed, mode 1: calib position

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("PID Control Motor");
    fillPortAction();
    Setup_Graph();
    SerialPort = new QSerialPort(this);
    timer_Uart = new QTimer(this);
    timer_Smoothing = new QTimer(this);
    connect(timer_Uart,SIGNAL(timeout()),this,SLOT(readData()));
    connect(timer_Smoothing,SIGNAL(timeout()),this,SLOT(smooth_Setpoint()));
    connect(ui->btn_exit,SIGNAL(clicked()),this,SLOT(close()));
}

void MainWindow::displayData(){
    float set_speed = pre_setpoint;
    float process_speed = speed_data.myfloat;
    float error_speed = set_speed - process_speed;
    float process_position = position_data.myfloat;
    displayText(ui->SpeedSV,QString::number(set_speed),20);
    displayText(ui->SpeedPV,QString::number(process_speed),20);
    displayText(ui->SpeedError,QString::number(error_speed),20);
    displayText(ui->PositionPV,QString::number(process_position),20);
    displayText(ui->PositionSV,"-",20);
    displayText(ui->PositionError,"-",20);
}

void MainWindow::readPIDgammadata(){
    QFile PIDgammafile(dir+"/pidgamma.txt");
    if (!PIDgammafile.open(QIODevice::ReadOnly | QIODevice::Text))
            return;
    while (!PIDgammafile.atEnd()) {
        QString line = PIDgammafile.readLine();
        QStringList list = line.split(" ",Qt::SkipEmptyParts);
        PID.Kp = list[0].toFloat();
        PID.Ki = list[1].toFloat();
        PID.Kd = list[2].toFloat();
        gamma.gamma1 = list[3].toFloat();
        gamma.gamma2 = list[4].toFloat();
        gamma.gamma3 = list[5].toFloat();
    }
    PIDgammafile.close();
}

void MainWindow::reference_model(){
    rt = 0.0952*setpoint + 0.9048*rt_1;
    ym = 1.9523332*ym_1 - 0.9531338*ym_2 + 0.0004035*rt_1 + 0.0003971*rt_2;
    ym_2 = ym_1;
    ym_1 = ym;
    rt_2 = rt_1;
    rt_1 = rt;
}

void MainWindow::readData(){
    static int cnt = 0;
    char checksum_Rx = 0;
    QByteArray byte_data = SerialPort->readAll();
    if(!byte_data.isEmpty()){
        if(byte_data.startsWith("$$DLCN,") && byte_data[16]=='\r' && byte_data[17]=='\n'){
            for(int k = 0; k < 4; k++){
                speed_data.mybyte[3-k] = byte_data[7+k];
                checksum_Rx += byte_data[7+k];
            }
            for(int k = 0; k < 4; k++){
                position_data.mybyte[3-k] = byte_data[11+k];
                checksum_Rx += byte_data[11+k];
            }
            if(checksum_Rx == byte_data[15])
                reference_model();
                RealTimeData();
                if (!(cnt++%15))
                    displayData();
                writeData();
        }
    }
}

void MainWindow::on_btn_mode_clicked(){
    if (!SerialPort->isOpen()) return;
    mode+=2;//++
    QByteArray txbuff;
    char checksum_Tx = 0;
    switch(mode){
        /*case 3:
            displayText(ui->btn_mode,"MRA_P",15);
            txbuff = "$ModeB,";
            send_data.myfloat = 0.0;
            for(int i = 0; i < 4; i++){
                txbuff[7+i] = send_data.mybyte[3-i];
                checksum_Tx += send_data.mybyte[3-i];
            }
            txbuff[11] = checksum_Tx;
            txbuff[12] = '\r';
            txbuff[13] = '\n';
            SerialPort->write(txbuff,14);
            break;*/
        case 2://2
            displayText(ui->btn_mode,"MRA_S",15);
            txbuff = "$ModeA,";
            send_data.myfloat = 0.0;
            for(int i = 0; i < 4; i++){
                txbuff[7+i] = send_data.mybyte[3-i];
                checksum_Tx += send_data.mybyte[3-i];
            }
            txbuff[11] = checksum_Tx;
            txbuff[12] = '\r';
            txbuff[13] = '\n';
            SerialPort->write(txbuff,14);
            break;
        /*case 1:
            displayText(ui->btn_mode,"POSI",15);
            txbuff = "$ModeP,";
            send_data.myfloat = 0.0;
            for(int i = 0; i < 4; i++){
                txbuff[7+i] = send_data.mybyte[3-i];
                checksum_Tx += send_data.mybyte[3-i];
            }
            txbuff[11] = checksum_Tx;
            txbuff[12] = '\r';
            txbuff[13] = '\n';
            SerialPort->write(txbuff,14);
            break;*/
        case 0:
        default:
            mode = 0;
            txbuff = "$ModeS,";
            send_data.myfloat = 0.0;
            for(int i = 0; i < 4; i++){
                txbuff[7+i] = send_data.mybyte[3-i];
                checksum_Tx += send_data.mybyte[3-i];
            }
            txbuff[11] = checksum_Tx;
            txbuff[12] = '\r';
            txbuff[13] = '\n';
            SerialPort->write(txbuff,14);
            displayText(ui->btn_mode,"PID_S",15);
            break;
    }
}

void MainWindow::writeData(){
    static bool flag = true;
    QTime now = QTime::currentTime();
    QString timestring = now.toString();
    QFile datafile(dir+"/data/data"+datetime.toString("hh_mm_dd_MM")+".txt");
    datafile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);
    QTextStream out(&datafile);
    if (flag){
        flag = false;
//        out << "time setpoint speed position\n";
    }
//    out << timestring << " " << QString::number(pre_setpoint,'f',2) << " " << QString::number(speed_data.myfloat,'f',2) << " " << QString::number(position_data.myfloat,'f',2) << "\n";
    out << QString::number(speed_data.myfloat,'f',2) << " ";
}

void MainWindow::RealTimeData(){
    static int yMax = 0;
    static int yMin = 0;
    static double temp_time = 0;
    static QTime time(QTime::currentTime());
    if(flag_PlotTimer){
        flag_PlotTimer = false;
        temp_time = time.elapsed()/1000.0;
        yMax = 0;
    }
    double key = time.elapsed()/1000.0;
    // add du lieu cho speed
    ui->customPlot->graph(0)->addData(key, speed_data.myfloat);
//    ui->customPlot->graph(1)->addData(key, pre_setpoint);
    ui->customPlot->graph(1)->addData(key, ym);
    ui->customPlot->graph(0)->rescaleValueAxis();
    if (key>30.0)
        ui->customPlot->xAxis->setRange(key-30.0, key);
    else
        ui->customPlot->xAxis->setRange(temp_time, key, Qt::AlignLeft);
    if(setpoint >= yMax)
        yMax = setpoint;
    if(speed_data.myfloat >= yMax)
        yMax = (int)speed_data.myfloat;
    if(setpoint <= yMin)
        yMin = setpoint;
    if(speed_data.myfloat <= yMin)
        yMin = (int)speed_data.myfloat;
    ui->customPlot->yAxis->setRange(-400, 400);
    ui->customPlot->replot();

    // add du lieu cho position
    ui->customPlot2->graph(0)->addData(key, position_data.myfloat);
    if (key>30.0)
        ui->customPlot2->xAxis->setRange(key-30.0, key);
    else
        ui->customPlot2->xAxis->setRange(temp_time, key, Qt::AlignLeft);
    ui->customPlot2->replot();
    // calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    lastFpsKey = key;
    frameCount = 0;
}

void MainWindow::Setup_Graph(){
    ui->customPlot->addGraph(); // blue line
    ui->customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    ui->customPlot->addGraph(); // red line
    ui->customPlot->graph(1)->setPen(QPen(QColor(255, 110, 40)));
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    ui->customPlot->xAxis->setTicker(timeTicker);
    ui->customPlot->axisRect()->setupFullAxesBox();
    ui->customPlot->yAxis->setRange(0,15);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    ui->customPlot->xAxis->setLabel("Time (seconds)");
    ui->customPlot->yAxis->setLabel("Speed (rpm)");


    ui->customPlot2->addGraph(); // blue line
    ui->customPlot2->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    ui->customPlot2->addGraph(); // red line
    ui->customPlot2->graph(1)->setPen(QPen(QColor(255, 110, 40)));
    ui->customPlot2->xAxis->setTicker(timeTicker);
    ui->customPlot2->axisRect()->setupFullAxesBox();
    ui->customPlot2->yAxis->setRange(-180,180);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->customPlot2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot2->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot2->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot2->yAxis2, SLOT(setRange(QCPRange)));
    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    ui->customPlot2->xAxis->setLabel("Time (seconds)");
    ui->customPlot2->yAxis->setLabel("Position (deg)");
}

void MainWindow::fillPortAction() {
    const auto infos = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo &info : infos){
        ui->ComUart->addItem(info.portName());
    }
}

void MainWindow::on_actionExit_triggered(){
    SerialPort->close();
    this->close();
}

void MainWindow::on_btn_setUart_clicked(){
    SerialPort->setPortName(ui->ComUart->currentText());
    SerialPort->setBaudRate(QSerialPort::Baud115200);
    SerialPort->setDataBits(static_cast<QSerialPort::DataBits>(QSerialPort::Data8));
    SerialPort->setParity(static_cast<QSerialPort::Parity>(QSerialPort::NoParity));
    SerialPort->setStopBits(static_cast<QSerialPort::StopBits>(QSerialPort::TwoStop));
    SerialPort->setFlowControl(static_cast<QSerialPort::FlowControl>(QSerialPort::NoFlowControl));
    SerialPort->open(QIODevice::ReadWrite);
    connect(SerialPort, &QSerialPort::readyRead, this,&MainWindow::readData);
    timer_Uart->start(25);
    // Interval 0 means to refresh as fast as possible
    readPIDgammadata();
    displayText(ui->Kp_Input,QString::number(PID.Kp),15);
    displayText(ui->Ki_Input,QString::number(PID.Ki),15);
    displayText(ui->Kd_Input,QString::number(PID.Kd),15);
    displayText(ui->gamma1_Input,QString::number(gamma.gamma1),15);
    displayText(ui->gamma2_Input,QString::number(gamma.gamma2),15);
    displayText(ui->gamma3_Input,QString::number(gamma.gamma3),15);
    setPID();
    setgamma();
}
void MainWindow::on_btn_closeUart_clicked(){
    SerialPort->close();
}

void MainWindow::on_btn_clear_clicked(bool){ //clear btn
    if (!SerialPort->isOpen()) return;
    flag_PlotTimer = true;
    ui->customPlot->removeGraph(0);
    ui->customPlot->removeGraph(1);
    ui->customPlot2->removeGraph(0);
    ui->customPlot2->removeGraph(1);
    Setup_Graph();
}

void MainWindow::smooth_Setpoint(){
    char checksum_Tx = 0;
    QByteArray txbuff;
    txbuff="$SPEED,";
    pre_setpoint += resolution;
    count++;
    send_data.myfloat = pre_setpoint;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);
    if (count == smoothSteps){
        count = 0;
        timer_Smoothing->stop();
    }
}

void MainWindow::on_btn_setPoint_clicked(){
    if (!SerialPort->isOpen()) return;
    if (mode==1) return;
    setpoint = ui->RPMInput->toPlainText().toFloat();
    if(setpoint<=RPMmax && setpoint >=RPMmin)
        send_data.myfloat = setpoint;
    else if(setpoint > RPMmax){
        setpoint = RPMmax;
        send_data.myfloat = setpoint;
    } else if(setpoint < RPMmin){
        setpoint = RPMmin;
        send_data.myfloat = setpoint;
    }
//// De smooth he thong
//    resolution = (setpoint - pre_setpoint)/smoothSteps;
//    timer_Smoothing->start(50);


//// Khong smooth
    pre_setpoint = setpoint;
    char checksum_Tx = 0;
    QByteArray txbuff;
    txbuff="$SPEED,";
    send_data.myfloat = pre_setpoint;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);
}

void MainWindow::on_btn_setPosition_clicked(){
    if (!SerialPort->isOpen()) return;
    if (mode==0) return;
    float curPosi = ui->Position_Input->toPlainText().toFloat();
    QByteArray txbuff;
    txbuff = "$Set_p,";
    char checksum_Tx = 0;
    send_data.myfloat = curPosi;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);
}

void MainWindow::on_btn_setZero_clicked(){
    if (!SerialPort->isOpen()) return;
    QByteArray txbuff;
    char checksum_Tx = 0;
    //dung dong co
    stop_motor();
    QString zero = "0";
    displayText(ui->RPMInput,zero,32);
    //set vi tri luc do bang 0
    txbuff = "$Set_O,";
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = i;
        checksum_Tx += txbuff[7+i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);
}

void MainWindow::stop_motor(){
    setpoint = 0;
    pre_setpoint = 0;
    QByteArray txbuff;
    char checksum_Tx = 0;
    //set toc do bang 0 (dung dong co)
    txbuff="$SPEED,";
    send_data.myfloat = setpoint;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);
}

void MainWindow::displayText(QTextEdit* box, const QString &text, int fontsize){
    /*
     * Ex:
     * QString zero = "0";
     * displayNumber(ui->RPMInput,zero,32);
     * float afl = 21.3;
     * QString b = QString::number(afl);
     * displayNumber(ui->RPMInput,b,32);
     */
    box->setText(text);
    box->setAlignment(Qt::AlignCenter);
    QFont font = QFont("Impact");
    font.setPointSize(fontsize);
    box->setFont(font);
}
void MainWindow::displayText(QPushButton* box, const QString &text, int fontsize){
    box->setText(text);
    QFont font = QFont("Impact");
    font.setPointSize(fontsize);
    box->setFont(font);
}
void MainWindow::setPID(){
    QByteArray txbuff;
    txbuff = "$Set_P,";
    char checksum_Tx = 0;
    send_data.myfloat = PID.Kp;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    txbuff = "$Set_I,";
    send_data.myfloat = PID.Ki;
    checksum_Tx = 0;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    txbuff = "$Set_D,";
    send_data.myfloat = PID.Kd;
    checksum_Tx = 0;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    writePIDgammadata();
}

void MainWindow::setgamma(){
    QByteArray txbuff;
    txbuff = "$Set_1,";
    char checksum_Tx = 0;
    send_data.myfloat = gamma.gamma1;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    txbuff = "$Set_2,";
    send_data.myfloat = gamma.gamma2;
    checksum_Tx = 0;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    txbuff = "$Set_3,";
    send_data.myfloat = gamma.gamma3;
    checksum_Tx = 0;
    for(int i = 0; i < 4; i++){
        txbuff[7+i] = send_data.mybyte[3-i];
        checksum_Tx += send_data.mybyte[3-i];
    }
    txbuff[11] = checksum_Tx;
    txbuff[12] = '\r';
    txbuff[13] = '\n';
    SerialPort->write(txbuff,14);

    writePIDgammadata();
}

void MainWindow::writePIDgammadata(){
    QFile PIDgammafile(dir+"/pidgamma.txt");
    if(!PIDgammafile.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text))
        return;
    QString pid_data = QString::number(PID.Kp) + " " + QString::number(PID.Ki) + " " + QString::number(PID.Kd) + " " + QString::number(gamma.gamma1) + " " + QString::number(gamma.gamma2) + " " + QString::number(gamma.gamma3) + "\n";
    QTextStream out(&PIDgammafile);
    out << pid_data;
    PIDgammafile.flush();
    PIDgammafile.close();
}

void MainWindow::on_btn_setPID_clicked(){
    if (!SerialPort->isOpen()) return;
    if (mode==1) return;
    PID.Kp = ui->Kp_Input->toPlainText().toFloat();
    PID.Ki = ui->Ki_Input->toPlainText().toFloat();
    PID.Kd = ui->Kd_Input->toPlainText().toFloat();
    setPID();
}

void MainWindow::on_btn_setgamma_clicked(){
    if (!SerialPort->isOpen()) return;
    if (mode==1) return;
    gamma.gamma1 = ui->gamma1_Input->toPlainText().toFloat();
    gamma.gamma2 = ui->gamma2_Input->toPlainText().toFloat();
    gamma.gamma3 = ui->gamma3_Input->toPlainText().toFloat();
    qDebug() << gamma.gamma1;
    setgamma();
}

MainWindow::~MainWindow()
{
    stop_motor();
    SerialPort->close();
    delete ui;
}
