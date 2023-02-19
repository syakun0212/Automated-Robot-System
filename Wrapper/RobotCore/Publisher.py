from types import ModuleType
import logging
import fastdds


class SimplePubListener(fastdds.PublisherListener):
    def __init__(self):
        super().__init__()
        self.match = 0

    def on_publication_matched(self, dataWriter, info) -> None:

        if(info.current_count_change == 1):
            self.match = info.total_count
            logging.debug("Publisher Matched")
        elif(info.current_count_change == -1):
            match = info.total_count
            logging.debug("Publisher Unmatched")


class Publisher(object):
    def __init__(self, participant, topic, queueLength: int):
        self.participant = participant

        publisherQos = fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(publisherQos)
        self.publisher = self.participant.create_publisher(publisherQos)

        self.listener = SimplePubListener()

        writerQos = fastdds.DataWriterQos()
        historyQos = fastdds.HistoryQosPolicy()
        historyQos.depth = queueLength
        historyQos.kind = fastdds.KEEP_ALL_HISTORY_QOS
        writerQos.history(historyQos)

        self.publisher.get_default_datawriter_qos(writerQos)
        self.writer = self.publisher.create_datawriter(
            topic, writerQos, self.listener)

    def __del__(self):
        if(self.writer is not None):
            self.publisher.delete_datawriter(self.writer)

        if(self.publisher is not None):
            self.participant.delete_publisher(self.publisher)

    def publish(self, data) -> bool:
        if(self.matched()):
            self.writer.write(data)
            return True
        return False

    def matched(self) -> bool:
        return self.listener.match > 0