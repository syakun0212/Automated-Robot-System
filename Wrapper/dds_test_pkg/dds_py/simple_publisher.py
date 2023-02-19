import fastdds
import HelloWorld

# For Sample App
import sys
import time

class SimplePubListener(fastdds.PublisherListener):
    def __init__(self):
        super().__init__()
        self.match = 0

    def on_publication_matched(self, dataWriter, info):

        if(info.current_count_change == 1):
            self.match = info.total_count
            print("Publisher Matched")
        elif(info.current_count_change == -1):
            match = info.total_count
            print("Publisher Unmatched")

class SimplePublisher(object):

    def __init__(self):
        self.domain = 0
        self.factory = None
        self.type_support = None
        self.participant = None
        self.topic = None
        self.publisher = None
        self.writer = None
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

        publisher_qos = fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(publisher_qos)
        self.publisher = self.participant.create_publisher(publisher_qos)

        self.listener = SimplePubListener()

        writer_qos = fastdds.DataWriterQos()
        self.publisher.get_default_datawriter_qos(writer_qos)
        self.writer = self.publisher.create_datawriter(self.topic, writer_qos, self.listener)

        self.data = HelloWorld.HelloWorld()
        self.data.index(0)
        self.data.message("HelloWorld")

        return True

    def __del__(self):
        if(self.writer is not None):
            self.publisher.delete_datawriter(self.writer)

        if(self.publisher is not None):
            self.participant.delete_publisher(self.publisher)
        
        if(self.topic is not None):
            self.participant.delete_topic(self.topic)
        
        self.factory.delete_participant(self.participant)

    def publish(self):
        if(self.listener.match > 0):
            self.data.index(self.data.index() + 1)
            self.writer.write(self.data)
            return True
        return False

    def run(self, sample):
        sent = 0
        while(sent < sample):

            if(self.publish()):
                sent += 1
                print(f"Sent: [{self.data.index()}] {self.data.message()} ")
            time.sleep(1)

if __name__ == "__main__":
    topic = sys.argv[1]

    pub = SimplePublisher()

    sample = 10
    domain = 5

    if(pub.init(domain, topic)):
        try:
            pub.run(sample)
        except KeyboardInterrupt:
            print("Exiting")
    else:
        print("Initialization failed")

    del pub