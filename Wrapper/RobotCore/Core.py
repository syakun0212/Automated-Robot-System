from pydoc import classname
from types import FunctionType, ModuleType
from RobotCore.Publisher import Publisher
from RobotCore.Subscriber import Subscriber
import fastdds


class Core(object):

    def __init__(self, domain=0):
        self.domain = domain
        self.factory = fastdds.DomainParticipantFactory.get_instance()
        participant_qos = fastdds.DomainParticipantQos()

        self.factory.get_default_participant_qos(participant_qos)
        self.participant = self.factory.create_participant(
            self.domain, participant_qos)

        self.msgTypes = {}

        self.topics = {}

        self.publishers = []
        self.subscribers = []

        if(self.participant is None):
            raise RuntimeError("Initialization failed")

    def registerType(self, msgType: ModuleType) -> None:
        className = msgType.__name__.split(".")[-1]
        topicDataType = getattr(msgType, className + "PubSubType")()

        if(topicDataType is None):
            raise RuntimeError(
                f"Failed to get message type: {className}")

        topicDataType.setName(className)
        typeSupport = fastdds.TypeSupport(topicDataType)

        self.participant.register_type(typeSupport)

        self.msgTypes[className] = topicDataType

    def registerTopic(self, msgTypeName: str, topicName: str, queueLength: int) -> None:

        topicQos = fastdds.TopicQos()
        historyQos = fastdds.HistoryQosPolicy()
        historyQos.depth = queueLength
        historyQos.kind = fastdds.KEEP_ALL_HISTORY_QOS
        topicQos.history(historyQos)
        self.participant.get_default_topic_qos(topicQos)
        topic = self.participant.create_topic(
            topicName, self.msgTypes[msgTypeName].getName(), topicQos)

        self.topics[topicName] = topic

    def createPublisher(self, msgType: ModuleType, topicName: str, queueLength: int = 1) -> Publisher:
        className = msgType.__name__.split(".")[-1]
        if(msgType.__name__ not in self.msgTypes):
            self.registerType(msgType)

        if(topicName not in self.topics):
            self.registerTopic(className, topicName, queueLength)

        pub = Publisher(self.participant, self.topics[topicName], queueLength)
        self.publishers.append(pub)
        return pub

    def createSubscriber(self, msgType: ModuleType, topicName: str, callbackFunc: FunctionType, queueLength: int = 1):
        className = msgType.__name__.split(".")[-1]
        if(className not in self.msgTypes):
            self.registerType(msgType)

        if(topicName not in self.topics):
            self.registerTopic(className, topicName, queueLength)

        dataType = getattr(msgType, className)

        sub = Subscriber(
            self.participant, self.topics[topicName], dataType, callbackFunc, queueLength)
        self.subscribers.append(sub)
        return sub

    def __del__(self):
        # delete publisher
        if len(self.publishers) > 0:
            self.publishers.clear()
            del self.publishers[:]

        # delete subscriber
        if len(self.subscribers) > 0:
            self.subscribers.clear()
            del self.subscribers[:]

        # delete topic
        if len(self.topics) > 0:
            self.topics.clear()
            del self.topics

        # delete participant
        self.factory.delete_participant(self.participant)
