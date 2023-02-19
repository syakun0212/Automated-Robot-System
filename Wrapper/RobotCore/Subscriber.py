from types import FunctionType, ModuleType
import logging
import fastdds


class SimpleSubListener(fastdds.SubscriberListener):
    def __init__(self, dataType, callbackFunc):
        super().__init__()
        self.match = 0
        self.dataType = dataType
        self.callbackFunc = callbackFunc

    def on_subscription_matched(self, dataReader, info):
        '''
            For debugging
        '''
        if(info.current_count_change == 1):
            self.match = info.total_count
            logging.debug("Subscriber Matched")
        elif(info.current_count_change == -1):
            match = info.total_count
            logging.debug("Subscriber Unmatched")

    def on_data_available(self, dataReader: fastdds.DataReader):
        info = fastdds.SampleInfo()
        data = self.dataType()
        if(dataReader.take_next_sample(data, info)):
            if(info.valid_data):
                self.callbackFunc(data)


class Subscriber(object):
    def __init__(self, participant, topic, dataType, callbackFunc: FunctionType, queueLength: int):
        self.participant = participant

        subscriberQos = fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(subscriberQos)
        self.subscriber = self.participant.create_subscriber(subscriberQos)

        self.listener = SimpleSubListener(dataType, callbackFunc)

        readerQos = fastdds.DataReaderQos()
        historyQos = fastdds.HistoryQosPolicy()
        historyQos.depth = queueLength
        historyQos.kind = fastdds.KEEP_ALL_HISTORY_QOS
        readerQos.history(historyQos)

        self.subscriber.get_default_datareader_qos(readerQos)
        self.reader = self.subscriber.create_datareader(
            topic, readerQos, self.listener)

    def __del__(self):
        if(self.reader is not None):
            self.subscriber.delete_datareader(self.reader)

        if(self.subscriber is not None):
            self.participant.delete_subscriber(self.subscriber)

    def matched(self) -> bool:
        return self.listener.match > 0