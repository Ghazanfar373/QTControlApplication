#ifndef DDSINTERFACE_H
#define DDSINTERFACE_H


#include "ccpp_FDM.h"
#include "ccpp_dds_dcps.h"
#include "CheckStatus.h"
#include "QObject"
#include "QThread"
#include "QMetaType"
#include "FDMDcps_impl.h"
#include "TacticalAircraftsDcps_impl.h"


using namespace FDM;
using namespace TacticalAircrafts;
using namespace DDS;
using namespace std;


class DDSInterface: public QThread
{
   Q_OBJECT

public:
    DDSInterface();
    ~DDSInterface();


    void run();
   // void quit();
public:
signals:
   void dataRecieved(fdmdata);
   void dataRecievedTacticalAircrafts(TacticalAircraftsData);

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


    Topic_var fdmdataTopic;
    fdmdataTypeSupport_var fdmdataType;
    fdmdataDataReader_var fdmdataReader;

    Topic_var TacticalAircraftsDataTopic;
    TacticalAircraftsDataTypeSupport_var TacticalAircraftsDataType;
    TacticalAircraftsDataDataReader_var TacticalAircraftsDataReader;



    //void publishFDMData();

    fdmdata FDMData;
    TacticalAircraftsData TacticalAircraftsDataInstance;

//public slots:
    void readFDMDataTopic();
    void readTacticalAircraftsDataTopic();

private:
    ReturnCode_t status;
    CORBA::String_var partition;



};

#endif // DDSINTERFACE_H
