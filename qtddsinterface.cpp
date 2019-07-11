#include "qtddsinterface.h"


char *topicName[]={"FDMDATA","TACTICALAIRCRAFTS"};

DDSInterface::DDSInterface()
{

    cout<<"before DDSInterface"<<endl;

    //bool autodispose_unregistered_instances = false;
    createParticipant("DDSDomain","DDSDomain");
    cout<<"Participant Created!"<<endl;
    fdmdataType = new fdmdataTypeSupport();
    TacticalAircraftsDataType = new TacticalAircraftsDataTypeSupport();
    //DaqDataType = new DaqDataTypeSupport();

    createTopic(fdmdataType, topicName[0], fdmdataTopic);
    cout<<"fdmdata Topic Created!"<<endl;
    createTopic(TacticalAircraftsDataType,topicName[1],TacticalAircraftsDataTopic);
    cout<<"TacticalAircraftsData Topic Created!"<<endl;

//    createTopic(DaqDataType,topicName[2],DaqDataTopic);
//    cout<<"DaqData Topic Created!"<<endl;

    cout<<"Topics Created!"<<endl;
    createPublisher();
    cout<<"Publisher Created!"<<endl;
    createSubscriber();
    cout<<"Subscriber Created!"<<endl;
//    fdmdataWriter = FDMDataDataWriter::_narrow(
//            createDataWriter(autodispose_unregistered_instances, fdmdataTopic)
//            );
    fdmdataReader = fdmdataDataReader::_narrow(createDataReader(fdmdataTopic));
    cout<<"FDMData Reader Created!"<<endl;
    TacticalAircraftsDataReader = TacticalAircraftsDataDataReader::_narrow(createDataReader(TacticalAircraftsDataTopic));
    cout<<"TacticalAircraftsData Reader Created!"<<endl;
//    DaqDataReader = DaqDataDataReader::_narrow(createDataReader(DaqDataTopic));
//    cout<<"DataReader for Daq Data Created!"<<endl;

    qRegisterMetaType<fdmdata>("fdmdata");
    qRegisterMetaType<TacticalAircraftsData>("TacticalAircraftsData");

}

void DDSInterface::run(){

    while(1){
        msleep(8);
        //cout<<"DDS Thread is Running ..."<<endl;
        readFDMDataTopic();
        readTacticalAircraftsDataTopic();
        //readInstDataTopic();
    }
}



void DDSInterface::readFDMDataTopic(){

    fdmdataSeq fdmdatalist;
    SampleInfoSeq info;
    ReturnCode_t status = fdmdataReader->take(fdmdatalist, info, LENGTH_UNLIMITED,
                                                        ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE);


    //cout<<"Status: "<<status<<endl;
    if (status!=RETCODE_OK)
        return;
    FDMData = fdmdatalist[0];
    //cout<<fdmdata.position.lat_gc_deg<<endl;
    //qDebug("fdmdata recieved!");
    emit dataRecieved(FDMData);
    fdmdataReader->return_loan(fdmdatalist, info);

}

void DDSInterface::readTacticalAircraftsDataTopic()
{
    TacticalAircraftsDataSeq tacticalAircraftsDataList;
    SampleInfoSeq info;
    ReturnCode_t status = TacticalAircraftsDataReader->take(tacticalAircraftsDataList, info, LENGTH_UNLIMITED,
                                                        ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE);


    //cout<<"Status: "<<status<<endl;
    if (status!=RETCODE_OK)
        return;
    TacticalAircraftsDataInstance = tacticalAircraftsDataList[0];
    //qDebug("fdmdata recieved!");
    emit dataRecievedTacticalAircrafts(TacticalAircraftsDataInstance);
    TacticalAircraftsDataReader->return_loan(tacticalAircraftsDataList, info);
}



void DDSInterface::createParticipant(const char *domainName, const char * partitionName){
    dpf = DomainParticipantFactory::get_instance();
    checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
    participant = dpf->create_participant(domainName, PARTICIPANT_QOS_DEFAULT, NULL,
      STATUS_MASK_NONE);
    checkHandle(participant.in(),
      "DDS::DomainParticipantFactory::create_participant");
    partition = partitionName;
}


void DDSInterface::deleteParticipant(){
    status = dpf->delete_participant(participant.in());
         checkStatus(status, "DDS::DomainParticipant::delete_participant ");
}


void DDSInterface::createTopic(TypeSupport *ts, char *topicName, Topic_var &topic){


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

void DDSInterface::createPublisher(){
    DataReaderQos qos;
    qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    status = participant->get_default_publisher_qos(pub_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    pub_qos.partition.name.length(1);
    pub_qos.partition.name[0] = partition;

    publisher = participant->create_publisher(pub_qos, NULL, STATUS_MASK_NONE);
    checkHandle(publisher.in(), "DDS::DomainParticipant::create_publisher");
}

void DDSInterface::deletePublisher()
{
    status = participant->delete_publisher(publisher.in());
    checkStatus(status, "DDS::DomainParticipant::delete_publisher ");
}

void DDSInterface::createSubscriber()
{
  int status = participant->get_default_subscriber_qos(sub_qos);
  checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
  sub_qos.partition.name.length(1);
  sub_qos.partition.name[0] = partition;
  subscriber = participant->create_subscriber(sub_qos, NULL, STATUS_MASK_NONE);
  checkHandle(subscriber.in(), "DDS::DomainParticipant::create_subscriber");
}

void DDSInterface::deleteSubscriber()
{
  status = participant->delete_subscriber(subscriber);
  checkStatus(status, "DDS::DomainParticipant::delete_subscriber ");
}

DataWriter_ptr DDSInterface::createDataWriter(bool autodispose_unregistered_instances, Topic_var &topic){
    DataWriter_ptr writer;
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

void DDSInterface::deleteDataWriter(DataWriter_ptr writer){
    status = publisher->delete_datawriter(writer);
        checkStatus(status, "DDS::Publisher::delete_datawriter ");
}

DataReader_ptr DDSInterface::createDataReader(Topic_var &topic)
{
    DataReaderQos qos;
    subscriber->get_default_datareader_qos(qos);
    qos.history.depth = 5;
    qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
  //  qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    DataReader_ptr reader = subscriber->create_datareader(topic.in(),
    qos, NULL, STATUS_MASK_NONE);
    checkHandle(reader, "DDS::Subscriber::create_datareader ()");
    return reader;
}


void DDSInterface::deleteDataReader(DataReader_ptr reader)
{
    status = subscriber->delete_datareader(reader);
    checkStatus(status, "DDS::Subscriber::delete_datareader ");
}

DDSInterface::~DDSInterface(){
   // fdmdataReader->return_loan()
    deleteDataReader(fdmdataReader);
    deletePublisher();
    deleteSubscriber();
    deleteParticipant();
}
