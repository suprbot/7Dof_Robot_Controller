#include "messagetransfer.h"

#include <QDebug>



MessageTransfer::MessageTransfer()
{
    messageSocket = new QTcpSocket(this);
}

MessageTransfer::~MessageTransfer() { messageSocket->close(); }

void MessageTransfer::robotConnect(QString address, int port)
{
    messageSocket->connectToHost(address, port);

    if(messageSocket->waitForConnected(1000)){
        emit connectStatus(true, address, port);
        qDebug() << address << port << "connected";
    }
    else {
        emit connectStatus(false, address, port);
        qDebug() << address << port << "not connected";
    }
}


int MessageTransfer::transMsg()
{
    while(1){
        messageSocket->read((char*)&rc2tp, sizeof(rc2tp));
        emit getMessage();

        //if(rc2tp.done)tp2rc.exec=false;
        //qDebug() << tp2rc.exec << tp2rc.tarpos;

        messageSocket->write((const char*)&tp2rc, sizeof(tp2rc));
        messageSocket->waitForBytesWritten(100);
        msleep(100);
    }
    return 0;
}


void MessageTransfer::run() { transMsg(); }
