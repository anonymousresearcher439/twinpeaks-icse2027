import time
# import threading
from threading import Event, Thread
from typing import List
from dr_onboard_autonomy.states.collections.MessageQueue import MessageLookupTable, MessageQueue, RecordState

def test_lookup_table_init_with_one_message_type():
    lookup_table = MessageLookupTable(["test"])
    assert lookup_table._message_types == frozenset(["test"])
    assert lookup_table._lock is not None
    assert len(lookup_table._message_lookup_table) == 1

def test_lookup_table_init_with_zero_message_types():
    lookup_table = MessageLookupTable([])
    assert lookup_table._message_types == frozenset()
    assert lookup_table._lock is not None
    assert len(lookup_table._message_lookup_table) == 0

def test_lookup_table_init_with_realistic_message_types():
    message_types = {
        "battery",
        "compass_hdg",
        "imu",
        "position",
        "relative_altitude",
        "velocity",
        # "diagnostics",
        # "estimator_status",
        # "state"
        # "extended_state"
        # "rcin"
    }
    lookup_table = MessageLookupTable(message_types)
    assert lookup_table._message_types == frozenset(message_types)
    assert len(lookup_table._message_lookup_table) == len(message_types)
    assert lookup_table._lock is not None

def test_create_ref_message():
    message_types = {
        "battery",
        "compass_hdg",
        "imu",
        "position",
        "relative_altitude",
        "velocity",
        # "diagnostics",
        # "estimator_status",
        # "state"
        # "extended_state"
        # "rcin"
    }
    lookup_table = MessageLookupTable(message_types)
    assert lookup_table._message_types == frozenset(message_types)
    
    imu_msg = {
        "type": "imu",
        "data": 1
    }
    ref_msg = lookup_table.create_ref_message(imu_msg)
    assert lookup_table._message_lookup_table["imu"].state == RecordState.AVAILABLE
    assert ref_msg.get() == imu_msg
    assert lookup_table._message_lookup_table["imu"].state == RecordState.MISSING

def test_block_till_message_arrives():
    msg = {
        "type": "imu",
        "data": 1
    }
    q = MessageQueue()
    entry_event = Event()
    assert not entry_event.is_set()
    event = Event()
    assert not event.is_set()

    def get_a_message():
        entry_event.set()
        nonlocal q
        m = q.get()
        assert m == {
            "type": "imu",
            "data": 1
        }
        event.set()

    t = Thread(target=get_a_message)
    t.start()
    entry_event.wait(timeout=1)
    assert not event.is_set()
    # time.sleep(0.05)
    assert not event.is_set()
    q.put(msg)
    event.wait(timeout=1)

def test_many_block_till_message():
    class Consumer(Thread):
        def __init__(self, q: MessageQueue, expected_msg: dict):
            super().__init__()
            self.q = q
            self.start_event = Event()
            self.event = Event()
            self.msg = None
            self.expected_msg = expected_msg


        def run(self):
            self.start_event.set()
            self.msg = self.q.get()
            if self.msg == self.expected_msg:
                self.event.set()
    
    q = MessageQueue()
    msgs = [
        {"type": "imu", "data": 1},
        {"type": "position", "data": 3},
        {"type": "timer", "data": 4},
        {"type": "heartbeat", "data": 5},
    ]
    consumers = [ Consumer(q, msg) for msg in msgs ]
    for c in consumers:
        c.start()
    for c in consumers:
        c.start_event.wait(timeout=0.1)
    for m in msgs:
        q.put(m)
    for c in consumers:
        c.event.wait(timeout=1)

def test_one_consumer_many_producers():
    class Producer(Thread):
        def __init__(self, q: MessageQueue, msg: dict):
            super().__init__()
            self.q = q
            self.start_event = Event()
            self.event = Event()
            self.msg = msg

        def run(self):
            self.start_event.set()
            self.q.put(self.msg)
            self.event.set()
    
    q = MessageQueue()
    msgs = [
        {"type": "imu", "data": 1},
        {"type": "position", "data": 3},
        {"type": "timer", "data": 4},
        {"type": "heartbeat", "data": 5},
    ]
    producers = [ Producer(q, msg) for msg in msgs ]

    consumer_done_event = Event()
    def get_all_messages():
        nonlocal q
        nonlocal msgs
        nonlocal consumer_done_event
        all_msgs = set([m['type'] for m in msgs])
        for msg in msgs:
            m = q.get()
            assert m['type'] in all_msgs
        consumer_done_event.set()
    
    t = Thread(target=get_all_messages)
    t.start()

    for p in producers:
        p.start()
    for p in producers:
        p.start_event.wait(timeout=1)
    for p in producers:
        p.event.wait(timeout=1)

    consumer_done_event.wait(timeout=1)


def test_one_consumer_many_special_producers():
    q = MessageQueue()
    msg_types = ["battery", "compass_hdg", "imu", "position", "relative_altitude", "velocity"]
    msg_counts = [10, 9, 8, 7, 6, 5]
    
    
    consumer_done_event = Event()
    def get_all_messages():
        nonlocal q
        nonlocal msg_types
        nonlocal msg_counts
        nonlocal consumer_done_event

        expected_data = {}
        for msg_type, count in zip(msg_types, msg_counts):
            expected_data[msg_type] = count
        
        for _ in msg_types:
            m = q.get()
            msg_type = m['type']
            expected_value = expected_data[msg_type]
            actual_value = m['data']
            assert msg_type in expected_data
            assert expected_value == actual_value

        consumer_done_event.set()
    
    t = Thread(target=get_all_messages)
    t.start()

    for msg_type, count in zip(msg_types, msg_counts):
        for m in [{'type': msg_type, 'data': x} for x in range(count + 1)]:
            q.put(m)

    consumer_done_event.wait(timeout=1)

def test_one_consumer_reads_all_special_messages_then_new_special_messages_are_added():
    """
    In round one we add 5 position messages.
    Then we get a message to make sure we only receive the last one.

    In round two we add 5 more position messages.
    Then we get a message to make sure we only received the youngest one from round two.
    """
    q = MessageQueue()
    round_one = [{'type': 'position', 'data': x} for x in range(1, 5+1)]
    for m in round_one:
        q.put(m)
    last_msg = q.get()
    assert last_msg == round_one[-1]
    assert last_msg == {'type': 'position', 'data': 5}

    round_two = [{'type': 'position', 'data': x} for x in range(5, 10+1)]
    for m in round_two:
        q.put(m)
    last_msg = q.get()
    assert last_msg == round_two[-1]
    assert last_msg == {'type': 'position', 'data': 10}

def test_the_len_method_returns_the_number_of_messages_in_the_queue():
    q = MessageQueue()
    msgs = [
        {"type": "imu", "data": 1},
        {"type": "battery", "data": 2},
        {"type": "position", "data": 3},
        {"type": "timer", "data": 4},
        {"type": "heartbeat", "data": 5},
    ]
    assert len(q) == 0
    for m in msgs:
        q.put(m)
    assert len(q) == len(msgs)
    
    non_special_msgs = [
        {"type": "a", "data": 1},
        {"type": "b", "data": 2},
        {"type": "c", "data": 3},
    ]
    for m in non_special_msgs:
        q.put(m)
    assert len(q) == len(msgs) + len(non_special_msgs)

    # add some special messages
    special_msgs = [
        {"type": "battery", "data": 1},
        {"type": "imu", "data": 3},
        {"type": "position", "data": 4},
    ]
    for m in special_msgs:
        q.put(m)
    assert len(q) == len(msgs) + len(non_special_msgs) # we don't + len(special_msgs) because they are repeats