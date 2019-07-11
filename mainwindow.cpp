#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    InitializeApplication();
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Receiver %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    system("./startDomain.sh");
    cout<<"Before DDS Thread"<<endl;
    dds = new DDSInterface;
    cout<<"After DDS Thread"<<endl;
    dds->start(QThread::LowPriority);
    cout<<"After DDS Thread Starts"<<endl;
    connect(dds,SIGNAL(dataRecievedTacticalAircrafts(TacticalAircraftsData)),this,SLOT(OnDataReceivedTacticalAircrafts(TacticalAircraftsData)));
    connect(dds,SIGNAL(dataRecieved(fdmdata)),this,SLOT(OnDataRecieved(fdmdata)));

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Transmitter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    bool autodispose_unregistered_instances = false;
    createParticipant("DDSDomain","DDSDomain");
    cout<<"Participant Created!"<<endl;
    QTTacticalEnvironmentDataType = new QTTacticalEnvironmentDataTypeSupport();
    QTControlDataType = new  QtControlDataTypeSupport();


    createTopic(QTTacticalEnvironmentDataType,"QTTACTICALENVIRONMENT",QTTacticalEnvironmentDataTopic);
    createTopic(QTControlDataType, "QTCONTROLDATA", QTControlDataTopic);

    cout<<"Topics Created!"<<endl;
    createPublisher();
    cout<<"Publisher Created!"<<endl;
    createSubscriber();
    cout<<"Subscriber Created!"<<endl;


    QTTacticalEnvironmentDataWriter = QTTacticalEnvironmentDataDataWriter::_narrow(createDataWriter(autodispose_unregistered_instances,QTTacticalEnvironmentDataTopic));
    cout<<"QTTacticalEnvironmentData Created!"<<endl;
    QTControlDataWriter = QtControlDataDataWriter::_narrow(createDataWriter(autodispose_unregistered_instances,QTControlDataTopic));
    cout<<"QTControlData Created!"<<endl;

    connect(this,SIGNAL(TriggerTacticalEnvironment()),this,SLOT(PublishTacticalEnvironmentData()));
    connect(this,SIGNAL(TriggerPublishQTControlData()),this,SLOT(PublishQTControlData()));

}
QTTacticalEnvironmentData qtTacticalEnvironmentData;

void MainWindow::InitializeApplication()
{
ui->ElectricalSystem->removeTab(1);
ui->ElectricalSystem->removeTab(1);
ui->ElectricalSystem->removeTab(1);
    InitializeFDMReceiveData();
    InitializeTacticalEnvironmentData();
    InitializeQTControlData();
}
void MainWindow::InitializeFDMReceiveData()
{
    ui->TrueAirSpeedLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->GroundSpeedLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->VerticalVelocityLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->MachNumberLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->GValueLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->AngleOfAttackLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->SideSlipAngleLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->CurrentLatitudeLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->CurrentLongitudeLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->HeadingLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->AltitudeAboveGroundLevelLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->AltitudeAboveSeaLevelLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    ui->GeneratorStatusLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->GeneratorIndicatorLight->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->ACBusIndicatorLightLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->ACBusVoltageLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->DCBusIndicatorLightLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->DCBusVoltageLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->BatteryBusVoltageLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->PitchLevelLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->RollLevelLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->PhiDotLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->ThetaDotLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->PsiDotLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->throttleVal->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);



}

void MainWindow::InitializeTacticalEnvironmentData()
{
    //Tactical Environment
    //Aircraft 1
    ui->Aircraft1Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft1TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //Aircraft 2
    ui->Aircraft2Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft2TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //Aircraft 3
    ui->Aircraft3Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft3TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //Aircraft 4
    ui->Aircraft4Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft4TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //Aircraft 5
    ui->Aircraft5Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft5TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //Aircraft 6
    ui->Aircraft6Exists->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Name->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Altitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Heading->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Velocity->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Latitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6Longitude->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6ClimbRate->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6OnCollisionCourse->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    ui->Aircraft6TimeToCollision->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
}
void MainWindow::InitializeQTControlData()
{
    ThrottleCommand = 0.0;
    ElevatorCommand = 500.0;
    AileronCommand = 500.0;
    RudderCommand = 500.0;
    LeftBrakeCommand = 0.0;
    RightBrakeCommand = 0.0;
    FlapCommand = 0.0;
    EngineStartSwitch = false;
    WindSpeedCommand = 0.0;
    WindDirectionCommand = 0.0;
    //ElectricalSystem
    GeneratorSwitch = false;
    InverterSwitch = false;
    BatteryStatus = false;
}

void MainWindow::PublishQTControlData()
{
    QtControlData qtControlDataInstance;
    qtControlDataInstance.AileronCommand = AileronCommand;
    qtControlDataInstance.ElevatorCommand = ElevatorCommand;
    qtControlDataInstance.EngineStartSwitch = EngineStartSwitch;
    qtControlDataInstance.FlapCommand = FlapCommand;
    qtControlDataInstance.LeftBrakeCommand = LeftBrakeCommand;
    qtControlDataInstance.RightBrakeCommand = RightBrakeCommand;
    qtControlDataInstance.RudderCommand = RudderCommand;
    qtControlDataInstance.ThrottleCommand = ThrottleCommand;
    qtControlDataInstance.WindSpeedCommand = WindSpeedCommand;
    qtControlDataInstance.WindDirectionCommand = WindDirectionCommand;

    qtControlDataInstance.GeneratorSwitch = GeneratorSwitch;
    qtControlDataInstance.InverterSwitch = InverterSwitch;
    qtControlDataInstance.BatteryStatus = BatteryStatus;

    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTControlDataWriter->write(qtControlDataInstance, NULL);
    checkStatus(status, "QTControlDataWriter::write");
}

void MainWindow::PublishTacticalEnvironmentData()
{
    qtTacticalEnvironmentData.ID = 1;

    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::OnDataRecieved(fdmdata FDMData)
{
    ui->TrueAirSpeedLabel->setText(QString::number(FDMData.TrueAirSpeed));
    ui->GroundSpeedLabel->setText(QString::number(FDMData.GroundAirSpeed));
    ui->VerticalVelocityLabel->setText(QString::number(FDMData.VerticleVelocity_fpm));
    ui->MachNumberLabel->setText(QString::number(FDMData.MachNumber));
    ui->GValueLabel->setText(QString::number(FDMData.GValue));
    ui->AngleOfAttackLabel->setText(QString::number(FDMData.AngleOFAttack));
    ui->SideSlipAngleLabel->setText(QString::number(FDMData.SideSlip));
    ui->CurrentLatitudeLabel->setText(QString::number(FDMData.Latitude));
    ui->CurrentLongitudeLabel->setText(QString::number(FDMData.Longitude));
    ui->HeadingLabel->setText(QString::number(FDMData.Heading_deg));
    ui->AltitudeAboveGroundLevelLabel->setText(QString::number(FDMData.Altitude_ft));
    ui->AltitudeAboveSeaLevelLabel->setText(QString::number(FDMData.AltitudeAboveSeaLevel_ft));

    ui->GeneratorStatusLabel->setText(QString::number(FDMData.GeneratorStatus));
    ui->GeneratorIndicatorLight->setText(QString::number(FDMData.GeneratorIndicatorLight));
    ui->ACBusIndicatorLightLabel->setText(QString::number(FDMData.ACBusIndicatorLight));
    ui->ACBusVoltageLabel->setText(QString::number(FDMData.ACBusVoltage));
    ui->DCBusIndicatorLightLabel->setText(QString::number(FDMData.DCBusIndicatorLight));
    ui->DCBusVoltageLabel->setText(QString::number(FDMData.DCBusVoltage));
    ui->BatteryBusVoltageLabel->setText(QString::number(FDMData.BatteryVoltage));
    ui->PitchLevelLabel->setText(QString::number(FDMData.Pitch_deg));
    ui->RollLevelLabel->setText(QString::number(FDMData.Roll_deg));

    ui->PhiDotLabel->setText(QString::number(FDMData.phidot));
    ui->ThetaDotLabel->setText(QString::number(FDMData.thetadot));
    ui->PsiDotLabel->setText(QString::number(FDMData.psidot));

    ui->throttleVal->setText(QString::number(FDMData.BatteryVoltage));

}

void MainWindow::OnDataReceivedTacticalAircrafts(TacticalAircraftsData tacticalAircraftsDataInstance)
{
    ui->Aircraft1Exists->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftExists[0]));
    if (tacticalAircraftsDataInstance.TEAircraftName[0] == NULL)
    {
        ui->Aircraft1Name->setText("Unknown");
    }
    else
    {
        ui->Aircraft1Name->setText(QString::fromStdString(std::string(tacticalAircraftsDataInstance.TEAircraftName[0])));
    }
    ui->Aircraft1Velocity->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftVelocity[0]));
    ui->Aircraft1Heading->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftHeading[0]));
    ui->Aircraft1Altitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftAltitude[0]));
    ui->Aircraft1Latitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLatitude[0]));
    ui->Aircraft1Longitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLongitude[0]));
    ui->Aircraft1ClimbRate->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftRateOfClimb[0]));
    ui->Aircraft1OnCollisionCourse->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftOnCollissionCourse[0]));
    ui->Aircraft1TimeToCollision->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftTimeToCollision[0]));

    ui->Aircraft2Exists->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftExists[1]));
    if (tacticalAircraftsDataInstance.TEAircraftName[1] == NULL)
    {
        ui->Aircraft2Name->setText("Unknown");
    }
    else
    {
       ui->Aircraft2Name->setText(QString::fromStdString(std::string(tacticalAircraftsDataInstance.TEAircraftName[1])));
    }
    ui->Aircraft2Velocity->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftVelocity[1]));
    ui->Aircraft2Heading->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftHeading[1]));
    ui->Aircraft2Altitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftAltitude[1]));
    ui->Aircraft2Latitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLatitude[1]));
    ui->Aircraft2Longitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLongitude[1]));
    ui->Aircraft2ClimbRate->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftRateOfClimb[1]));
    ui->Aircraft2OnCollisionCourse->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftOnCollissionCourse[1]));
    ui->Aircraft3TimeToCollision->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftTimeToCollision[1]));

    ui->Aircraft3Exists->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftExists[2]));
    if (tacticalAircraftsDataInstance.TEAircraftName[2] == NULL)
    {
        ui->Aircraft3Name->setText("Unknown");
    }
    else
    {
       ui->Aircraft3Name->setText(QString::fromStdString(std::string(tacticalAircraftsDataInstance.TEAircraftName[2])));
    }
    ui->Aircraft3Velocity->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftVelocity[2]));
    ui->Aircraft3Heading->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftHeading[2]));
    ui->Aircraft3Altitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftAltitude[2]));
    ui->Aircraft3Latitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLatitude[2]));
    ui->Aircraft3Longitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLongitude[2]));
    ui->Aircraft3ClimbRate->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftRateOfClimb[2]));
    ui->Aircraft3OnCollisionCourse->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftOnCollissionCourse[2]));
    ui->Aircraft3TimeToCollision->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftTimeToCollision[2]));

    ui->Aircraft4Exists->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftExists[3]));
    if (tacticalAircraftsDataInstance.TEAircraftName[3] == NULL)
    {
        ui->Aircraft4Name->setText("Unknown");
    }
    else
    {
       ui->Aircraft4Name->setText(QString::fromStdString(std::string(tacticalAircraftsDataInstance.TEAircraftName[3])));
    }
    ui->Aircraft4Velocity->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftVelocity[3]));
    ui->Aircraft4Heading->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftHeading[3]));
    ui->Aircraft4Altitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftAltitude[3]));
    ui->Aircraft4Latitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLatitude[3]));
    ui->Aircraft4Longitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLongitude[3]));
    ui->Aircraft4ClimbRate->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftRateOfClimb[3]));
    ui->Aircraft4OnCollisionCourse->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftOnCollissionCourse[3]));
    ui->Aircraft4TimeToCollision->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftTimeToCollision[3]));

    ui->Aircraft5Exists->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftExists[4]));
    if (tacticalAircraftsDataInstance.TEAircraftName[4] == NULL)
    {
        ui->Aircraft5Name->setText("Unknown");
    }
    else
    {
       ui->Aircraft5Name->setText(QString::fromStdString(std::string(tacticalAircraftsDataInstance.TEAircraftName[4])));
    }
    ui->Aircraft5Velocity->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftVelocity[4]));
    ui->Aircraft5Heading->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftHeading[4]));
    ui->Aircraft5Altitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftAltitude[4]));
    ui->Aircraft5Latitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLatitude[4]));
    ui->Aircraft5Longitude->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftLongitude[4]));
    ui->Aircraft5ClimbRate->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftRateOfClimb[4]));
    ui->Aircraft5OnCollisionCourse->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftOnCollissionCourse[4]));
    ui->Aircraft5TimeToCollision->setText(QString::number(tacticalAircraftsDataInstance.TEAircraftTimeToCollision[4]));

}

MainWindow::~MainWindow()
{
    delete ui;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%For DDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%

void MainWindow::createParticipant(const char *domainName, const char * partitionName){
    dpf = DomainParticipantFactory::get_instance();
    checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
    participant = dpf->create_participant(domainName, PARTICIPANT_QOS_DEFAULT, NULL,
      STATUS_MASK_NONE);
    checkHandle(participant.in(),
      "DDS::DomainParticipantFactory::create_participant");
    partition = partitionName;
}


void MainWindow::deleteParticipant(){
    status = dpf->delete_participant(participant.in());
         checkStatus(status, "DDS::DomainParticipant::delete_participant ");
}

void MainWindow::createTopic(TypeSupport *ts, char *topicName, Topic_var &topic){


     CORBA::String_var typeName;
     typeName = ts->get_type_name();

     ts->register_type(participant.in(),typeName);
     //checkStatus(status, "register_type");
     Duration_t t;
     t.sec = 1;
     topic = participant->find_topic(topicName, t);
     status = participant->get_default_topic_qos(reliable_topic_qos);
     checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
     //reliable_topic_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
     //reliable_topic_qos.durability.kind = TRANSIENT_DURABILITY_QOS;

     /* Make the tailored QoS the new default. */
     status = participant->set_default_topic_qos(reliable_topic_qos);
     checkStatus(status, "DDS::DomainParticipant::set_default_topic_qos");

     /* Use the changed policy when defining the HelloWorld topic */
     if(topic == NULL)
     {
         cout <<"topic not found creating topic"<<endl;
     topic = participant->create_topic(topicName, typeName, reliable_topic_qos,
             NULL, STATUS_MASK_NONE);
     checkHandle(topic.in(), "DDS::DomainParticipant::create_topic ()");
     } else cout << "topic found!"<<endl;

}

void MainWindow::createPublisher(){
    DataReaderQos qos;
    qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
//    qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
//    qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    status = participant->get_default_publisher_qos(pub_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    pub_qos.partition.name.length(1);
    pub_qos.partition.name[0] = partition;

    publisher = participant->create_publisher(pub_qos, NULL, STATUS_MASK_NONE);
    checkHandle(publisher.in(), "DDS::DomainParticipant::create_publisher");
}

void MainWindow::deletePublisher()
{
    status = participant->delete_publisher(publisher.in());
    checkStatus(status, "DDS::DomainParticipant::delete_publisher ");
}

void MainWindow::createSubscriber()
{
  int status = participant->get_default_subscriber_qos(sub_qos);
  checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
  sub_qos.partition.name.length(1);
  sub_qos.partition.name[0] = partition;
  subscriber = participant->create_subscriber(sub_qos, NULL, STATUS_MASK_NONE);
  checkHandle(subscriber.in(), "DDS::DomainParticipant::create_subscriber");
}

void MainWindow::deleteSubscriber()
{
  status = participant->delete_subscriber(subscriber);
  checkStatus(status, "DDS::DomainParticipant::delete_subscriber ");
}

DataWriter_ptr MainWindow::createDataWriter(bool autodispose_unregistered_instances, Topic_var &topic){
    DataWriter_ptr writer;
    DataWriterQos qos;
    qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
    status = publisher->get_default_datawriter_qos(dw_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    status = publisher->copy_from_topic_qos(dw_qos, reliable_topic_qos);
    checkStatus(status, "DDS::Publisher::copy_from_topic_qos");
    // Set autodispose to false so that you can start
    // the subscriber after the publisher
    dw_qos.writer_data_lifecycle.autodispose_unregistered_instances =
    autodispose_unregistered_instances;
    writer = publisher->create_datawriter(topic.in(), dw_qos, NULL,
    STATUS_MASK_NONE);
    checkHandle(writer, "DDS::Publisher::create_datawriter");
    return writer;
}

void MainWindow::deleteDataWriter(DataWriter_ptr writer){
    status = publisher->delete_datawriter(writer);
        checkStatus(status, "DDS::Publisher::delete_datawriter ");
}

DataWriter_ptr MainWindow::createDataWriterIOS(bool autodispose_unregistered_instances, Topic_var &topic){
    DataWriter_ptr writer;
    DataWriterQos qos;
    qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
    qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    status = publisher->get_default_datawriter_qos(dw_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    status = publisher->copy_from_topic_qos(dw_qos, reliable_topic_qos);
    checkStatus(status, "DDS::Publisher::copy_from_topic_qos");
    // Set autodispose to false so that you can start
    // the subscriber after the publisher
    dw_qos.writer_data_lifecycle.autodispose_unregistered_instances =
    autodispose_unregistered_instances;
    writer = publisher->create_datawriter(topic.in(), dw_qos, NULL,
    STATUS_MASK_NONE);
    checkHandle(writer, "DDS::Publisher::create_datawriter");
    return writer;
}

void MainWindow::deleteDataWriterIOS(DataWriter_ptr writer){
    status = publisher->delete_datawriter(writer);
        checkStatus(status, "DDS::Publisher::delete_datawriter ");
}


DataReader_ptr MainWindow::createDataReader(Topic_var &topic)
{
    DataReaderQos qos;
    subscriber->get_default_datareader_qos(qos);
    qos.history.depth = 5;
    qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
  //  qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    DataReader_ptr reader = subscriber->create_datareader(topic.in(),
    qos, NULL, STATUS_MASK_NONE);
    checkHandle(reader, "DDS::Subscriber::create_datareader ()");
    return reader;
}

void MainWindow::deleteDataReader(DataReader_ptr reader)
{
    status = subscriber->delete_datareader(reader);
    checkStatus(status, "DDS::Subscriber::delete_datareader ");
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void MainWindow::on_CreateAircraft1Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[0] = true;
    qtTacticalEnvironmentData.TEAircraftName[0] = ui->TEAircraft1Name->text().toStdString().c_str();
    qtTacticalEnvironmentData.TEAircraftVelocity[0] = ui->TEAircraft1Velocity->value();
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[0] = ui->TEAircraft1TotalNumberOfPath->value();
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[0] = ui->TEAircraft1RateOfClimb->value();
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[0] = ui->TEAircraft1RateOfTurn->value();
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][0] = ui->TEAircraft1Path1Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][0] = ui->TEAircraft1Path1Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][0] = ui->TEAircraft1Path1Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][0] = ui->TEAircraft1Path1Head->text().toDouble();
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][1] = ui->TEAircraft1Path2Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][1] = ui->TEAircraft1Path2Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][1] = ui->TEAircraft1Path2Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][1] = ui->TEAircraft1Path2Head->text().toDouble();
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][2] = ui->TEAircraft1Path3Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][2] = ui->TEAircraft1Path3Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][2] = ui->TEAircraft1Path3Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][2] = ui->TEAircraft1Path3Head->text().toDouble();
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][3] = ui->TEAircraft1Path4Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][3] = ui->TEAircraft1Path4Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][3] = ui->TEAircraft1Path4Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][3] = ui->TEAircraft1Path4Head->text().toDouble();


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");

}

void MainWindow::on_DeleteAircraft1Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[0] = false;
    qtTacticalEnvironmentData.TEAircraftName[0] = "Aircraft deleted";
    qtTacticalEnvironmentData.TEAircraftVelocity[0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[0] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[0] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[0] = 0;
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][0] = 0.0;
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][1] = 0.0;
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][2] = 0.0;
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[0][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[0][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[0][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[0][3] = 0.0;


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_CreateAircraft2Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[1] = true;
    qtTacticalEnvironmentData.TEAircraftName[1] = ui->TEAircraft2Name->text().toStdString().c_str();
    qtTacticalEnvironmentData.TEAircraftVelocity[1] = ui->TEAircraft2Velocity->value();
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[1] = ui->TEAircraft2TotalNumberOfPath->value();
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[1] = ui->TEAircraft2RateOfClimb->value();
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[1] = ui->TEAircraft2RateOfTurn->value();
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][0] = ui->TEAircraft2Path1Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][0] = ui->TEAircraft2Path1Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][0] = ui->TEAircraft2Path1Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][0] = ui->TEAircraft2Path1Head->text().toDouble();
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][1] = ui->TEAircraft2Path2Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][1] = ui->TEAircraft2Path2Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][1] = ui->TEAircraft2Path2Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][1] = ui->TEAircraft2Path2Head->text().toDouble();
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][2] = ui->TEAircraft2Path3Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][2] = ui->TEAircraft2Path3Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][2] = ui->TEAircraft2Path3Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][2] = ui->TEAircraft2Path3Head->text().toDouble();
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][3] = ui->TEAircraft2Path4Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][3] = ui->TEAircraft2Path4Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][3] = ui->TEAircraft2Path4Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][3] = ui->TEAircraft2Path4Head->text().toDouble();


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_DeleteAircraft2Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[1] = false;
    qtTacticalEnvironmentData.TEAircraftName[1] = "Aircraft deleted";
    qtTacticalEnvironmentData.TEAircraftVelocity[1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[1] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[1] = 0.0;
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][0] = 0.0;
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][1] = 0.0;
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][2] = 0.0;
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[1][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[1][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[1][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[1][3] = 0.0;


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_CreateAircraft3Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[2] = true;
    qtTacticalEnvironmentData.TEAircraftName[2] = ui->TEAircraft3Name->text().toStdString().c_str();
    qtTacticalEnvironmentData.TEAircraftVelocity[2] = ui->TEAircraft3Velocity->value();
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[2] = ui->TEAircraft3TotalNumberOfPath->value();
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[2] = ui->TEAircraft3RateOfClimb->value();
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[2] = ui->TEAircraft3RateOfTurn->value();
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][0] = ui->TEAircraft3Path1Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][0] = ui->TEAircraft3Path1Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][0] = ui->TEAircraft3Path1Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][0] = ui->TEAircraft3Path1Head->text().toDouble();
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][1] = ui->TEAircraft3Path2Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][1] = ui->TEAircraft3Path2Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][1] = ui->TEAircraft3Path2Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][1] = ui->TEAircraft3Path2Head->text().toDouble();
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][2] = ui->TEAircraft3Path3Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][2] = ui->TEAircraft3Path3Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][2] = ui->TEAircraft3Path3Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][2] = ui->TEAircraft3Path3Head->text().toDouble();
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][3] = ui->TEAircraft3Path4Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][3] = ui->TEAircraft3Path4Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][3] = ui->TEAircraft3Path4Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][3] = ui->TEAircraft3Path4Head->text().toDouble();


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_DeleteAircraft3Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[2] = false;
    qtTacticalEnvironmentData.TEAircraftName[2] = "Aircraft deleted";
    qtTacticalEnvironmentData.TEAircraftVelocity[2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[2] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[2] = 0.0;
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][0] = 0.0;
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][1] = 0.0;
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][2] = 0.0;
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[2][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[2][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[2][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[2][3] = 0.0;


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_CreateAircraft4Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[3] = true;
    qtTacticalEnvironmentData.TEAircraftName[3] = ui->TEAircraft4Name->text().toStdString().c_str();
    qtTacticalEnvironmentData.TEAircraftVelocity[3] = ui->TEAircraft4Velocity->value();
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[3] = ui->TEAircraft4TotalNumberOfPath->value();
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[3] = ui->TEAircraft4RateOfClimb->value();
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[3] = ui->TEAircraft4RateOfTurn->value();
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][0] = ui->TEAircraft4Path1Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][0] = ui->TEAircraft4Path1Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][0] = ui->TEAircraft4Path1Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][0] = ui->TEAircraft4Path1Head->text().toDouble();
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][1] = ui->TEAircraft4Path2Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][1] = ui->TEAircraft4Path2Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][1] = ui->TEAircraft4Path2Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][1] = ui->TEAircraft4Path2Head->text().toDouble();
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][2] = ui->TEAircraft4Path3Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][2] = ui->TEAircraft4Path3Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][2] = ui->TEAircraft4Path3Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][2] = ui->TEAircraft4Path3Head->text().toDouble();
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][3] = ui->TEAircraft4Path4Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][3] = ui->TEAircraft4Path4Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][3] = ui->TEAircraft4Path4Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][3] = ui->TEAircraft4Path4Head->text().toDouble();


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_DeleteAircraft4Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[3] = false;
    qtTacticalEnvironmentData.TEAircraftName[3] = "Aircraft deleted";
    qtTacticalEnvironmentData.TEAircraftVelocity[3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[3] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[3] = 0.0;
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][0] = 0.0;
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][1] = 0.0;
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][2] = 0.0;
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[3][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[3][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[3][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[3][3] = 0.0;


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_CreateAircraft5Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[4] = true;
    qtTacticalEnvironmentData.TEAircraftName[4] = ui->TEAircraft5Name->text().toStdString().c_str();
    qtTacticalEnvironmentData.TEAircraftVelocity[4] = ui->TEAircraft5Velocity->value();
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[4] = ui->TEAircraft5TotalNumberOfPath->value();
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[4] = ui->TEAircraft5RateOfClimb->value();
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[4] = ui->TEAircraft5RateOfTurn->value();
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][0] = ui->TEAircraft5Path1Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][0] = ui->TEAircraft5Path1Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][0] = ui->TEAircraft5Path1Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][0] = ui->TEAircraft5Path1Head->text().toDouble();
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][1] = ui->TEAircraft5Path2Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][1] = ui->TEAircraft5Path2Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][1] = ui->TEAircraft5Path2Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][1] = ui->TEAircraft5Path2Head->text().toDouble();
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][2] = ui->TEAircraft5Path3Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][2] = ui->TEAircraft5Path3Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][2] = ui->TEAircraft5Path3Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][2] = ui->TEAircraft5Path3Head->text().toDouble();
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][3] = ui->TEAircraft5Path4Lat->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][3] = ui->TEAircraft5Path4Long->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][3] = ui->TEAircraft5Path4Alt->text().toDouble();
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][3] = ui->TEAircraft5Path4Head->text().toDouble();


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_DeleteAircraft5Button_clicked()
{
    qtTacticalEnvironmentData.ID = 1;
    qtTacticalEnvironmentData.CreatAircraft[4] = false;
    qtTacticalEnvironmentData.TEAircraftName[4] = "Aircraft deleted";
    qtTacticalEnvironmentData.TEAircraftVelocity[4] = 0.0;
    qtTacticalEnvironmentData.TEAircraftTotalNumberOfPoints[4] = 0;
    qtTacticalEnvironmentData.TEAircraftRateOfClimb[4] = 0.0;
    qtTacticalEnvironmentData.TEAircraftRateOFTurn[4] = 0.0;
    // Path 1
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][0] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][0] = 0.0;
    // Path 2
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][1] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][1] = 0.0;
    //Path 3
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][2] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][2] = 0.0;
    //Path 4
    qtTacticalEnvironmentData.TEAircraftPathLatitude[4][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathLongitude[4][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathAltitude[4][3] = 0.0;
    qtTacticalEnvironmentData.TEAircraftPathHeading[4][3] = 0.0;


    cout<<"Published.........."<<endl;
    ReturnCode_t status = QTTacticalEnvironmentDataWriter->write(qtTacticalEnvironmentData, NULL);
    checkStatus(status, "QTTacticalEnvironmentDataWriter::write");
}

void MainWindow::on_ThrottleCommand_actionTriggered(int action)
{
    ThrottleCommand = ui->ThrottleCommand->value();
    PublishQTControlData();
}

void MainWindow::on_AileronCommand_actionTriggered(int action)
{
    AileronCommand = ui->AileronCommand->value();
    PublishQTControlData();
}

void MainWindow::on_RudderCommand_actionTriggered(int action)
{
    RudderCommand = ui->RudderCommand->value();
    PublishQTControlData();
}

void MainWindow::on_ElevatorCommand_actionTriggered(int action)
{
    ElevatorCommand = ui->ElevatorCommand->value();
    PublishQTControlData();
}

void MainWindow::on_FlapCommand_actionTriggered(int action)
{
    FlapCommand = ui->FlapCommand->value()/100.0;
    PublishQTControlData();
}

void MainWindow::on_LeftBrakeCommand_actionTriggered(int action)
{
    LeftBrakeCommand = ui->LeftBrakeCommand->value()/100.0;
    PublishQTControlData();
}

void MainWindow::on_RightBrakeCommand_actionTriggered(int action)
{
    RightBrakeCommand = ui->RightBrakeCommand->value()/100.0;
    PublishQTControlData();
}

void MainWindow::on_EngineStartSwitch_clicked()
{
    EngineStartSwitch = ui->EngineStartSwitch->isChecked();
    PublishQTControlData();
}

void MainWindow::on_WindSpeedSlider_actionTriggered(int action)
{
    WindSpeedCommand = ui->WindSpeedSlider->value();
    PublishQTControlData();
}

void MainWindow::on_WindDirectionSlider_sliderReleased()
{
    WindDirectionCommand = ui->WindDirectionSlider->value();
    PublishQTControlData();
}



void MainWindow::on_GeneratorSwitch_clicked()
{
    GeneratorSwitch = ui->GeneratorSwitch->isChecked();
    PublishQTControlData();
}

void MainWindow::on_InverterSwitch_clicked()
{
    InverterSwitch = ui->InverterSwitch->isChecked();
    PublishQTControlData();
}

void MainWindow::on_BatteryStatus_clicked()
{
    BatteryStatus = ui->BatteryStatus->isChecked();
    PublishQTControlData();
}
