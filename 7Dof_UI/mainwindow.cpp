#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sstream>
#include <iomanip>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    msgTsf = new MessageTransfer();
    connect(msgTsf, SIGNAL(getMessage()), this, SLOT(getMessage()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_connect2ServerButton_clicked()
{
    msgTsf->robotConnect(ui->addressEdit->text(), ui->portEdit->text().toInt());

    msgTsf->start();
}

void MainWindow::getMessage()
{
    std::ostringstream ost_joint[7];
    std::ostringstream ost_end[6];
    for(int i=0;i<7;i++)
    {
        ost_joint[i].str("");
        ost_joint[i]<<std::fixed<<std::setprecision(2)<<std::setfill('0')<<std::setw(6)<<rc2tp.jointPos[i];
    }

    for(int i=0;i<6;i++)
    {
        ost_end[i].str("");
        ost_end[i]<<std::fixed<<std::setprecision(2)<<std::setfill('0')<<std::setw(6)<<rc2tp.endPos[i];
    }
//    qDebug()<<rc2tp.jointPos[0]<<rc2tp.jointPos[1]<<rc2tp.jointPos[2]<<rc2tp.jointPos[3]<<rc2tp.jointPos[4]<<rc2tp.jointPos[5]<<rc2tp.jointPos[6];
//    qDebug()<<rc2tp.endPos[0]<<rc2tp.endPos[1]<<rc2tp.endPos[2]<<rc2tp.endPos[3]<<rc2tp.endPos[4]<<rc2tp.endPos[5];

    ui->lcdNum_axis_1->display(QString::fromStdString(ost_joint[0].str()));
    ui->lcdNum_axis_2->display(QString::fromStdString(ost_joint[1].str()));
    ui->lcdNum_axis_3->display(QString::fromStdString(ost_joint[2].str()));
    ui->lcdNum_axis_4->display(QString::fromStdString(ost_joint[3].str()));
    ui->lcdNum_axis_5->display(QString::fromStdString(ost_joint[4].str()));
    ui->lcdNum_axis_6->display(QString::fromStdString(ost_joint[5].str()));
    ui->lcdNum_axis_7->display(QString::fromStdString(ost_joint[6].str()));

    ui->lcdNum_end_x->display(QString::fromStdString(ost_end[0].str()));
    ui->lcdNum_end_y->display(QString::fromStdString(ost_end[1].str()));
    ui->lcdNum_end_z->display(QString::fromStdString(ost_end[2].str()));
    ui->lcdNum_end_rx->display(QString::fromStdString(ost_end[3].str()));
    ui->lcdNum_end_ry->display(QString::fromStdString(ost_end[4].str()));
    ui->lcdNum_end_rz->display(QString::fromStdString(ost_end[5].str()));
}

void MainWindow::on_axis_1_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[0]=1;
}

void MainWindow::on_axis_1_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[0]=0;
}


void MainWindow::on_axis_1_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[0]=-1;
}

void MainWindow::on_axis_1_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[0]=0;
}

void MainWindow::on_axis_2_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[1]=1;
}

void MainWindow::on_axis_2_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[1]=0;
}

void MainWindow::on_axis_2_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[1]=-1;
}

void MainWindow::on_axis_2_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[1]=0;
}

void MainWindow::on_axis_3_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[2]=1;
}

void MainWindow::on_axis_3_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[2]=0;
}

void MainWindow::on_axis_3_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[2]=-1;
}

void MainWindow::on_axis_3_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[2]=0;
}

void MainWindow::on_end_x_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[0]=1;
}

void MainWindow::on_end_x_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[0]=0;
}

void MainWindow::on_end_x_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[0]=-1;
}

void MainWindow::on_end_x_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[0]=0;
}

void MainWindow::on_end_y_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[1]=1;
}

void MainWindow::on_end_y_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[1]=0;
}

void MainWindow::on_end_y_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[1]=-1;
}

void MainWindow::on_end_y_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[1]=0;
}

void MainWindow::on_end_z_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[2]=1;
}

void MainWindow::on_end_z_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[2]=0;
}

void MainWindow::on_end_z_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[2]=-1;
}

void MainWindow::on_end_z_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[2]=0;
}

void MainWindow::on_axis_4_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[3]=1;
}

void MainWindow::on_axis_4_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[3]=0;
}


void MainWindow::on_axis_4_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[3]=-1;
}

void MainWindow::on_axis_4_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[3]=0;
}

void MainWindow::on_axis_5_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[4]=1;
}

void MainWindow::on_axis_5_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[4]=0;
}


void MainWindow::on_axis_5_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[4]=-1;
}

void MainWindow::on_axis_5_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[4]=0;
}

void MainWindow::on_axis_6_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[5]=1;
}

void MainWindow::on_axis_6_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[5]=0;
}


void MainWindow::on_axis_6_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[5]=-1;
}

void MainWindow::on_axis_6_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[5]=0;
}
void MainWindow::on_axis_7_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[6]=1;
}

void MainWindow::on_axis_7_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[6]=0;
}

void MainWindow::on_axis_7_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=1;
    tp2rc.jogjoint[6]=-1;
}

void MainWindow::on_axis_7_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogjoint[6]=0;
}

void MainWindow::on_end_rx_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[3]=1;
}

void MainWindow::on_end_rx_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[3]=0;
}

void MainWindow::on_end_rx_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[3]=-1;
}

void MainWindow::on_end_rx_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[3]=0;
}

void MainWindow::on_end_ry_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[4]=1;
}

void MainWindow::on_end_ry_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[4]=0;
}

void MainWindow::on_end_ry_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[4]=-1;
}

void MainWindow::on_end_ry_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[4]=0;
}

void MainWindow::on_end_rz_fw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[5]=1;
}

void MainWindow::on_end_rz_fw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[5]=0;
}

void MainWindow::on_end_rz_bw_pressed()
{
    tp2rc.exec=true;
    tp2rc.jogMode=2;
    tp2rc.jogend[5]=-1;
}

void MainWindow::on_end_rz_bw_released()
{
    tp2rc.exec=false;
    tp2rc.jogMode=0;
    tp2rc.jogend[5]=0;
}
