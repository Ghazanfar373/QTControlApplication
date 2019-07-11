#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qtddsinterface.h"
#include "ccpp_FDM.h"
#include "ccpp_QTControl.h"
#include "ccpp_dds_dcps.h"
#include "CheckStatus.h"
#include "QObject"
#include "QMetaType"
#include "ccpp_QTTactEnvironment.h"
#include "ccpp_TacticalAircrafts.h"

using namespace FDM;
using namespace QTControl;
using namespace DDS;
using namespace QTTacticalEnvironment;
using namespace  TacticalAircrafts;
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void InitializeApplication();
    void InitializeFDMReceiveData();
    void InitializeTacticalEnvironmentData();
    void InitializeQTControlData();

public slots:
    void OnDataRecieved(fdmdata);
    void OnDataReceivedTacticalAircrafts(TacticalAircraftsData);
    void PublishTacticalEnvironmentData();
    void PublishQTControlData();
public:
signals:
   void TriggerPublishQTControlData();
   void TriggerTacticalEnvironment();

   private slots:
   void on_CreateAircraft1Button_clicked();

   void on_DeleteAircraft1Button_clicked();

   void on_CreateAircraft2Button_clicked();

   void on_DeleteAircraft2Button_clicked();

   void on_CreateAircraft3Button_clicked();

   void on_DeleteAircraft3Button_clicked();

   void on_CreateAircraft4Button_clicked();

   void on_DeleteAircraft4Button_clicked();

   void on_CreateAircraft5Button_clicked();

   void on_DeleteAircraft5Button_clicked();


   void on_ThrottleCommand_actionTriggered(int action);

   void on_AileronCommand_actionTriggered(int action);

   void on_RudderCommand_actionTriggered(int action);

   void on_ElevatorCommand_actionTriggered(int action);

   void on_FlapCommand_actionTriggered(int action);

   void on_LeftBrakeCommand_actionTriggered(int action);

   void on_RightBrakeCommand_actionTriggered(int action);

   void on_WindSpeedSlider_actionTriggered(int action);

   void on_EngineStartSwitch_clicked();

   void on_WindDirectionSlider_sliderReleased();

   void on_GeneratorSwitch_clicked();

   void on_InverterSwitch_clicked();

   void on_BatteryStatus_clicked();

private:

    void createParticipant(const char *domainName, const char * partitionName);
    void deleteParticipant();
    void createTopic(TypeSupport *ts, char *topicName, Topic_var &topic);
    void createPublisher();
    void deletePublisher();
    void createSubscriber();
    void deleteSubscriber();
    DataWriter_ptr createDataWriter(bool autodispose_unregistered_instances, Topic_var &topic);
    void deleteDataWriter(DataWriter_ptr writer);

    DataWriter_ptr createDataWriterIOS(bool autodispose_unregistered_instances, Topic_var &topic);
    void deleteDataWriterIOS(DataWriter_ptr writer);

    DataReader_ptr createDataReader(Topic_var &topic);
    void deleteDataReader(DataReader_ptr reader);

    DomainParticipantFactory_var dpf;
    DomainParticipant_var participant;

    Publisher_var publisher;
    Subscriber_var subscriber;

    /* QosPolicy holders */
    TopicQos reliable_topic_qos;
    TopicQos setting_topic_qos;
    PublisherQos pub_qos;
    DataWriterQos dw_qos;
    SubscriberQos sub_qos;


    Topic_var QTTacticalEnvironmentDataTopic;
    QTTacticalEnvironmentDataTypeSupport_var QTTacticalEnvironmentDataType;
    QTTacticalEnvironmentDataDataWriter_var QTTacticalEnvironmentDataWriter;

    Topic_var QTControlDataTopic;
    QtControlDataTypeSupport_var QTControlDataType;
    QtControlDataDataWriter_var QTControlDataWriter;



    //void publishFDMData();

    fdmdata FDMData;
    TacticalAircraftsData TacticalAircraftsDataInstance;

private:
    ReturnCode_t status;
    CORBA::String_var partition;
    Ui::MainWindow *ui;
    QThread* ReceiveThread;
    DDSInterface* dds;

    double ThrottleCommand;
    double ElevatorCommand;
    double AileronCommand;
    double RudderCommand;
    double LeftBrakeCommand;
    double RightBrakeCommand;
    double FlapCommand;
    bool EngineStartSwitch;
    double WindSpeedCommand;
    double WindDirectionCommand;

    //ElectricalSystem
    bool GeneratorSwitch;
    bool InverterSwitch;
    bool BatteryStatus;

};

#endif // MAINWINDOW_H
