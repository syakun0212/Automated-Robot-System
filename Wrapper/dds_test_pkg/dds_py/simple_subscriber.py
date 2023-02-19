import fastdds
import HelloWorld

# For Sample App
import sys
import time

class SimpleSubListener(fastdds.SubscriberListener):
    def __init__(self, callback_func):
        super().__init__()
        self.match = 0
        self.callback_func = callback_func

    def on_subscription_matched(self, dataReader, info):

        if(info.current_count_change == 1):
            self.match = info.total_count
            print("Subscriber Matched")
        elif(info.current_count_change == -1):
            match = info.total_count
            print("Subscriber Unmatched")

    def on_data_available(self, dataReader: fastdds.DataReader):
        info = fastdds.SampleInfo()
        data = HelloWorld.HelloWorld()
        if(dataReader.take_next_sample(data, info)):
            if(info.valid_data):
                self.callback_func(data)

class SimpleSubscriber(object):

    def __init__(self):
        self.domain = 0
        self.factory = None
        self.type_support = None
        self.participant = None
        self.topic = None
        self.subscriber = None
        self.reader = None
        self.listener = None

    def init(self, domain, topic):
        self.domain = domain
        self.factory = fastdds.DomainParticipantFactory.get_instance()
        participant_qos = fastdds.DomainParticipantQos()

        self.factory.get_default_participant_qos(participant_qos)
        self.participant = self.factory.create_participant(self.domain, participant_qos)

        if(self.participant is None):
            return False

        topic_data_type = HelloWorld.HelloWorldPubSubType()
        topic_data_type.setName("HelloWorldDataType")
        self.type_support = fastdds.TypeSupport(topic_data_type)
        self.participant.register_type(self.type_support)

        topic_qos = fastdds.TopicQos()
        self.participant.get_default_topic_qos(topic_qos)
        self.topic = self.participant.create_topic(topic, topic_data_type.getName(), topic_qos)

        subscriber_qos = fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(subscriber_qos)
        self.subscriber = self.participant.create_subscriber(subscriber_qos)

        self.listener = SimpleSubListener(self.callback)

        reader_qos = fastdds.DataReaderQos()
        self.subscriber.get_default_datareader_qos(reader_qos)
        self.reader = self.subscriber.create_datareader(self.topic, reader_qos, self.listener)

        return True

    def __del__(self):
        if(self.reader is not None):
            self.subscriber.delete_datareader(self.reader)

        if(self.subscriber is not None):
            self.participant.delete_subscriber(self.subscriber)
        
        if(self.topic is not None):
            self.participant.delete_topic(self.topic)
        
        self.factory.delete_participant(self.participant)
    
    def callback(self, data: HelloWorld.HelloWorld):
        print(f"Received: {data.message()} : {data.index()}")

if __name__ == "__main__":

    topic = sys.argv[1]


    sub = SimpleSubscriber()

    domain = 5

    if(sub.init(domain, topic)):
        try:
            while(True):
                time.sleep(5)
        except KeyboardInterrupt:
            print("Exiting")
    else:
        print("Initialization failed")

    del sub