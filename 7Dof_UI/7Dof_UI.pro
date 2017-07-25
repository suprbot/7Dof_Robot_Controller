#-------------------------------------------------
#
# Project created by QtCreator 2016-01-25T10:52:25
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 7Dof_UI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    messagetransfer.cpp \
    global_variables.cpp
    global_variables.cpp

HEADERS  += mainwindow.h \
    messagetransfer.h \
    global_variables.h

FORMS    += mainwindow.ui
