#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "messagetransfer.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    MessageTransfer *msgTsf;

private slots:
    void on_connect2ServerButton_clicked();

    void getMessage();

    void on_axis_1_fw_pressed();

    void on_axis_1_fw_released();

    void on_axis_2_fw_pressed();

    void on_axis_1_bw_pressed();

    void on_axis_1_bw_released();

    void on_axis_2_fw_released();

    void on_axis_2_bw_pressed();

    void on_axis_2_bw_released();

    void on_axis_3_fw_pressed();

    void on_axis_3_fw_released();

    void on_axis_3_bw_pressed();

    void on_axis_3_bw_released();

    void on_axis_4_fw_pressed();

    void on_axis_4_fw_released();

    void on_axis_5_fw_pressed();

    void on_axis_5_fw_released();

    void on_axis_6_fw_pressed();

    void on_axis_6_fw_released();

    void on_axis_7_fw_pressed();

    void on_axis_7_fw_released();

    void on_axis_4_bw_pressed();

    void on_axis_4_bw_released();

    void on_axis_5_bw_pressed();

    void on_axis_5_bw_released();

    void on_axis_6_bw_pressed();

    void on_axis_6_bw_released();

    void on_axis_7_bw_pressed();

    void on_axis_7_bw_released();

    void on_end_x_fw_pressed();

    void on_end_x_fw_released();

    void on_end_x_bw_pressed();

    void on_end_x_bw_released();

    void on_end_y_fw_pressed();

    void on_end_y_fw_released();

    void on_end_y_bw_pressed();

    void on_end_y_bw_released();

    void on_end_z_fw_pressed();

    void on_end_z_fw_released();

    void on_end_z_bw_pressed();

    void on_end_z_bw_released();

    void on_end_rx_fw_pressed();

    void on_end_rx_fw_released();

    void on_end_rx_bw_pressed();

    void on_end_rx_bw_released();

    void on_end_ry_fw_pressed();

    void on_end_ry_fw_released();

    void on_end_ry_bw_pressed();

    void on_end_ry_bw_released();

    void on_end_rz_fw_pressed();

    void on_end_rz_fw_released();

    void on_end_rz_bw_pressed();

    void on_end_rz_bw_released();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
