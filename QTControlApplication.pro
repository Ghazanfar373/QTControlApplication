xz#-------------------------------------------------
#
# Project created by QtCreator 2016-04-12T21:11:15
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QTControlApplication
TEMPLATE = app

INCLUDEPATH += . \
               ../jsbsim/src/DDSInterface \
               ../../HDE/x86.linux2.6/include/dcps/C++/SACPP \
               ../../HDE/x86.linux2.6/include/sys \
               ../../HDE/x86.linux2.6/include

SOURCES += main.cpp\
        mainwindow.cpp \
        ../jsbsim/src/DDSInterface/FDM.cpp \
        ../jsbsim/src/DDSInterface/FDMDcps.cpp \
        ../jsbsim/src/DDSInterface/FDMDcps_impl.cpp \
        ../jsbsim/src/DDSInterface/FDMSplDcps.cpp \
        ../jsbsim/src/DDSInterface/QTControl.cpp \
        ../jsbsim/src/DDSInterface/QTControlDcps.cpp \
        ../jsbsim/src/DDSInterface/QTControlDcps_impl.cpp \
        ../jsbsim/src/DDSInterface/QTControlSplDcps.cpp \
        ../jsbsim/src/DDSInterface/QTTactEnvironment.cpp \
        ../jsbsim/src/DDSInterface/QTTactEnvironmentDcps.cpp \
        ../jsbsim/src/DDSInterface/QTTactEnvironmentDcps_impl.cpp \
        ../jsbsim/src/DDSInterface/QTTactEnvironmentSplDcps.cpp \
        ../jsbsim/src/DDSInterface/TacticalAircrafts.cpp \
        ../jsbsim/src/DDSInterface/TacticalAircraftsDcps.cpp \
        ../jsbsim/src/DDSInterface/TacticalAircraftsDcps_impl.cpp \
        ../jsbsim/src/DDSInterface/TacticalAircraftsSplDcps.cpp \
        ../jsbsim/src/DDSInterface/CheckStatus.cpp \
        qtddsinterface.cpp

HEADERS  += mainwindow.h \
            ../jsbsim/src/DDSInterface/ccpp_FDM.h \
            ../jsbsim/src/DDSInterface/FDM.h \
            ../jsbsim/src/DDSInterface/FDMDcps.h \
            ../jsbsim/src/DDSInterface/FDMDcps_impl.h \
            ../jsbsim/src/DDSInterface/FDMSplDcps.h \
            ../jsbsim/src/DDSInterface/ccpp_QTControl.h \
            ../jsbsim/src/DDSInterface/QTControl.h \
            ../jsbsim/src/DDSInterface/QTControlDcps.h \
            ../jsbsim/src/DDSInterface/QTControlDcps_impl.h \
            ../jsbsim/src/DDSInterface/QTControlSplDcps.h \
            ../jsbsim/src/DDSInterface/ccpp_QTTactEnvironment.h \
            ../jsbsim/src/DDSInterface/QTTactEnvironment.h \
            ../jsbsim/src/DDSInterface/QTTactEnvironmentDcps.h \
            ../jsbsim/src/DDSInterface/QTTactEnvironmentDcps_impl.h \
            ../jsbsim/src/DDSInterface/QTTactEnvironmentSplDcps.h \
            ../jsbsim/src/DDSInterface/ccpp_TacticalAircrafts.h \
            ../jsbsim/src/DDSInterface/TacticalAircrafts.h \
            ../jsbsim/src/DDSInterface/TacticalAircraftsDcps.h \
            ../jsbsim/src/DDSInterface/TacticalAircraftsDcps_impl.h \
            ../jsbsim/src/DDSInterface/TacticalAircraftsSplDcps.h \
            ../jsbsim/src/DDSInterface/CheckStatus.h \
            qtddsinterface.h

FORMS    += mainwindow.ui
LIBS +=  -L/home/sunny/HDE/x86.linux2.6/lib/ -lcmjni -lcmxml -lcommonserv -ldcpsgapi -ldcpsjni -ldcpssacpp -ldcpssac -ldcpssaj -lddsconfparser -lddsconf -lddsdatabase -lddskernel -lddsosnet -lddsos -lddsserialization -lddsuser -lddsutil
