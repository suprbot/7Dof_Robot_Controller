#ifndef MESSAGETRANSFER_H
#define MESSAGETRANSFER_H
#include <QWidget>
#include <QThread>
#include <QTcpSocket>

#include "global_variables.h"

class MessageTransfer : public QThread
{
    Q_OBJECT
public:
    MessageTransfer();
    ~MessageTransfer();

    QTcpSocket *messageSocket;

    void robotConnect(QString address, int port);

    int transMsg();

private:
    void run();

signals:
    void connectStatus(bool, QString, int);
    void getMessage();
};

#endif // MESSAGETRANSFER_H
