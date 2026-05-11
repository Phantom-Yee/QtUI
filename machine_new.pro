#-------------------------------------------------
#
# Project created by QtCreator 2026-05-09T10:27:47
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = machine_new
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

# ===== PathPlanning DLL =====
INCLUDEPATH += $$PWD/../PathPlanning
LIBS += -L$$PWD/../PathPlanning/bin -lpathplanning

# Copy DLL next to executable after build
win32 {
    QMAKE_POST_LINK += $$quote(cmd /c copy /y "$$PWD\\..\\PathPlanning\\bin\\pathplanning.dll" "$$OUT_PWD\\pathplanning.dll")
}
