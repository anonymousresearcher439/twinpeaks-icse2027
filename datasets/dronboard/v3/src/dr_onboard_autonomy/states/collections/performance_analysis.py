import queue
import sqlite3
import threading
import time
import uuid

from collections import namedtuple
from typing import Optional
from pathlib import Path

import rospy

from .MessageQueue import MessageQueueImpl, Message


class PerformanceAnalysisQueue(MessageQueueImpl):
    """
    A specialized message queue that measures performance data. A state can use this MessageQueue implementation when we want to measures performance data. This is useful for diagnosing some types of issues and comparing implementations. This can also aid in code optimization, and it can help us determine if the CPU is overloaded.

    This queue is particularly useful for diagnosing performance issues, such as CPU overload, where the processor cannot keep up with the rate that messages are added to the queue.
     
    The PerformanceAnalysisQueue measures two things:

        1. the time it takes to process each message
        2. the time interval between consecutive messages of the same type
    
    The PerformanceAnalysisQueue saves a record of each measurement in a sqlite database (More on that later).
    
    We measure how long it takes to process each message by tapping into the `get()` method. The timer starts just before a message is returned by `get()`, and the timer stops the next time the `get()` method is called. We can then create a measurement record with the time difference and other details.

    We measure the time interval between consecutive messages of the same type by tapping into the `put()` method.
    when a message is added to the queue, we save the time in memory. The next time a message of the same type is added to the queue, we create a measurment record with the time difference and other details.

    All of our measurement records are _eventually_ saved to a database. Since our states process Hundreds or even thousands of messages per second, we don't want to save each record to the database as soon as it's measured. Instead, we temporarily save the records in memory and periodically flush them to the database. But these functions are handled by the PerfDataStore class.

    Why do we measure these two things?
    We can use this information to compare the rate at which we can process each type of message to the rate that we receive each type of message.  We can determine if a given type of message is being added to the queue faster than we can process it. This is a sign that the CPU is overloaded.

    Furthermore other types of analysis can be done on this data. For example, we can determine the average time it takes to process all types of messages. We can also determine the average period between all type of message. We can compare these two values to see if the CPU is overloaded.

    Lastly since we're making individual measurements we can look for anomalies. For example, we can look for messages that take an unusually long time to process.
    """
    def __init__(self, state, db: Optional['DatabaseManager']=None):
        global _database
        super().__init__()
        if not db:
            db = _database
        self._database = db
        self.state_name = state.name 
        self.state_type = state.__class__.__name__
        self.uuid = uuid.uuid4()
        
        # TODO what if this module wasn't properly Initialized and _database is None?
        # in that case we should probably raise an exception
        # because we can't save any data and if you don't want to save
        # performance data then you should use a different MessageQueue implementation
        self.state_id = db.create_state_record(self.uuid, self.state_name, self.state_type)

        self.processing_record_builder = ProcessingRecordBuilder(self.state_id)
        self.input_record_builder = InputRecordBuilder(self.state_id)
        
    
    def get(self) -> Message:
        """Return the next message from the queue.
        This will also save a measurement record about how long the previous message took to process (if possible).

        The very first time this method is called, it will not save a measurement record. This is because there is no previous message to measure. But every time after that, it will create a measurement record.
        """
        now = time.perf_counter_ns() # this is the time when the previous task was finished
        if self.processing_record_builder.has_pending_record():
            # the lines in this `if` statement are potentially long enough
            # operations that they could throw off our measurements. So we need
            # to carefully avoid including them in our measurements.
            record = self.processing_record_builder.complete_record(now)
            self.save_processing_record(record)

        result = super().get()

        # now that all the potentially long operations are done, we can record
        # the time again. We want to make sure that the time it took to save the measurements
        # is not included in our task measurements. So we read another time value
        t = time.perf_counter_ns() # this is the time when the next task is started.
        # lets update our memory with the new start time this is a lightweight
        # operation we just update some fields in the object
        self.processing_record_builder.start_new_record(result["type"], t)
        return result
    
    def put(self, message: Message) -> None:
        """Add a message to the queue.
        This will also save a measurement record.
        This record indicates the type of message and the timestamp.
        If possible the record will specify how long it's been since the last message of this type was added to the queue.
        If this is the first time we've seen this type of message, then the record will not include this time interval.
        """
        now = time.perf_counter_ns()
        record = self.input_record_builder.create_record(message["type"], now)
        self.save_input_record(record)
        super().put(message)
        # we measure the time again because we've just finished adding it to the queue
        # we don't want the time it took for us to record performance data to be included in the measurement
        now = time.perf_counter_ns()
        self.input_record_builder.update(message["type"], now)
    
    def finish_processing(self):
        """This method is called when the state is about to exit.
        We use this opportunity to save any pending measurement records.
        This way we can record how long the final message took to process.
        """
        now = time.perf_counter_ns()
        if self.processing_record_builder.has_pending_record():
            record = self.processing_record_builder.complete_record(now)
            self.save_processing_record(record)
    
    def save_processing_record(self, record: 'ProcessingRecord'):
        if self._database:
            self._database.save_processing_record(record)
        

    def save_input_record(self, record: 'InputRecord'):
        if self._database:
            self._database.save_input_record(record)
    


# When we measure how long a message takes to process, we create a ProcessingRecord.
# The easiest way to understand its fields is with this sentence:
#
# At `timestamp` our state (identified by `state_name` and `state_type`) started
# processing a message of type `message_type` and it took `delta_t_ns` seconds to do it.
#
# - timestamp is an integer representing the time in nanoseconds. It's relative
#   to the timestamp recorded by the first call to time.perf_counter_ns() in the program.
# - state_name is the name of the current state. It's a string. This is determined
#   by the mission
# - state_type is the type of the current state. It's a string. It's the class
#   name of the current state.
# - message_type is the type of the message being processed. It's a string. It's
#   provided by the message sender object that was the data source for this message.
#   The dr_onboard_autonomy.message_senders module defines our message sender classes.
# - delta_t_ns is the time it took to process the message in nanoseconds. It's
#   an integer. It's the difference between the timestamp recorded by the first
#   call to time.perf_counter_ns() when the program started working on the
#   message and the time recorded by the second call to time.perf_counter_ns()
#   the program was done.
ProcessingRecord = namedtuple('ProcessingRecord', [
    'timestamp',
    'state_id',
    'message_type',
    'delta_t_ns',
])


class ProcessingRecordBuilder:
    """This class is used internally to make it easier to build ProcessingRecords.

    The PerformanceAnalysisQueue needs to manage some state as it runs.
    This class encapsulates that state and provides a simple interface for us.
    """
    def __init__(self, state_id):
        self.state_id = state_id
        self.timestamp = None
        self.message_type = None

    def start_new_record(self, message_type: str, t_ns: int):
        """
        Args:
            message_type: the type of message that we just finished processing.
            t: the time when this record was started. recommend using time.perf_counter_ns() to get this value.
        """
        self.timestamp = t_ns
        self.message_type = message_type

    def complete_record(self, t_ns: int) -> ProcessingRecord:
        """Given the time when the message was finished processing, create a
        ProcessingRecord using the data that we've collected so far.

        PRECONDITION: there must be a pending record. That is, you must have previously called start_new_record() and not yet called complete_record(). The PRECONDITION is not checked.

        You can be certain that the PRECONDITION is met if has_pending_record() returns True.

        Args:
            t_ns: the time when the message was finished processing. recommend using time.perf_counter_ns() to get this value.
        """
        result = ProcessingRecord(
            timestamp=self.timestamp, # start time
            state_id=self.state_id,
            message_type=self.message_type,
            delta_t_ns=t_ns - self.timestamp
        )
        self.timestamp = None
        self.message_type = None
        return result

    def has_pending_record(self) -> bool:
        return self.message_type is not None


# When we measure the time interval between consecutive messages of the same type,
# we create an InputRecord. The easiest way to understand its fields is with this sentence:
#
# At `timestamp` our state (identified by `state_name` and `state_type`) received a message of type `message_type` and it's been `delta_t_ns` seconds since we received the last message of this type.
#

InputRecord = namedtuple('InputRecord', ['timestamp', 'state_id', 'message_type', 'delta_t_ns'])
"""
- timestamp is an integer representing the time in nanoseconds. It's relative to the timestamp recorded by the first call to time.perf_counter_ns() in the program.
- state_name is the name of the current state. It's a string. This is determined by the mission
- state_type is the type of the current state. It's a string. It's the class name of the current state.
- message_type is the type of the message being processed. It's a string. It's provided by the message sender object that was the data source for this message.
- delta_t_ns is the time interval between the current message and the previous message of the same type. It's an integer. It's the difference between the timestamp recorded by the first call to time.perf_counter_ns() when the program received the previous message and the followup call to time.perf_counter_ns() when it received the current message. If this is the first time we've received a message of this type, then it's ok for this value to be None.
"""

class InputRecordBuilder:
    def __init__(self, state_id):
        self.state_id = state_id
        self.lookup_table = {}
    
    def create_record(self, message_type: str, t_ns: int) -> Optional[InputRecord]:
        """There are two possible cases:

        1. this is the first time we've seen this type of message
        
        2. this is not the first time we've seen this type of message
        
        In the first case we return None. In the second caes we will create an
        InputRecord for the message_type and return it

        Args:
            message_type: the type of message that we just received.
            t_ns: the time when the message was received. Recommend using time.perf_counter_ns() to get this value.
        """
        delta_t = None

        if message_type in self.lookup_table:
            prev_t_ns = self.lookup_table[message_type]
            delta_t = t_ns - prev_t_ns
        
        return InputRecord(
            timestamp=t_ns,
            state_id=self.state_id,
            message_type=message_type,
            delta_t_ns=delta_t
        )
    
    def update(self, message_type: str, t_ns: int):
        self.lookup_table[message_type] = t_ns
            

AppBenchmarkRecord = namedtuple('AppBenchmarkRecord', ['run_id', 'start_time', 'comment'])

CREATE_TABLES_SQL = """
-- Creating the AppBenchmarkRecords table
CREATE TABLE IF NOT EXISTS AppBenchmarkRecords (
    run_id INTEGER PRIMARY KEY ASC,
    uuid TEXT,
    start_time INTEGER NOT NULL,
    comment TEXT
);

-- Creating the table for the State instances
-- Each state we build will have it's details saved in this table
CREATE TABLE IF NOT EXISTS States (
    state_id INTEGER PRIMARY KEY ASC,
    uuid TEXT,
    name TEXT,
    type TEXT,
    run_id INTEGER,
    FOREIGN KEY(run_id) REFERENCES AppBenchmarkRecords(run_id)
);

-- Creating the ProcessingRecords table
CREATE TABLE IF NOT EXISTS ProcessingRecords (
    t INTEGER,
    state_id INTEGER,
    message_type TEXT,
    delta_t INTEGER,
    run_id INTEGER,
    FOREIGN KEY(state_id) REFERENCES States(state_id),
    FOREIGN KEY(run_id) REFERENCES AppBenchmarkRecords(run_id)
);

-- Creating the InputRecords table
CREATE TABLE IF NOT EXISTS InputRecords (
    t INTEGER,
    state_id INTEGER,
    message_type TEXT,
    delta_t INTEGER,
    run_id INTEGER,
    FOREIGN KEY(run_id) REFERENCES AppBenchmarkRecords(run_id)
);
"""

class PerfDataStore:
    
    def __init__(self, sqlite_connection, flush_size=5000):
        self.conn = sqlite_connection
        self._processing_records = []
        self._input_records = []
        self.app_benchmark_record = None
        self.run_id = None
        self.flush_size = flush_size
    
    def create_tables(self):
        self.conn.executescript(CREATE_TABLES_SQL)
        self.conn.commit()
    
    def create_app_benchmark_record(self, start_time: int, comment: str):
        """
        We will create a record in the AppBenchmarkRecords table. This record
        All of the performance data that we collect will be associated with this record.
        This record represents a specific instance when we ran the application.

        Note that the AppBenchmarkRecord table specifies `run_id` as the primary key.
        But this column is an alias for the rowid column. The rowid column is the
        built-in primary key column you get by default with sqlite. 

        This method accesses the run_id using the curosor.lastrowid property.
        So I think it's important to note that this value is indeed the same as
        our `run_id`. 
        """
        assert start_time is not None
        app_uuid = uuid.uuid4()
        insert_sql = 'INSERT INTO AppBenchmarkRecords (uuid, start_time, comment) VALUES (?, ?, ?)'
        record = (str(app_uuid), start_time, comment)

        cursor = self.conn.cursor()
        
        # Insert the record into the database
        cursor.execute(insert_sql, record)

        # Retrieve the run_id of the inserted record
        run_id = cursor.lastrowid

        # Commit the changes
        self.conn.commit()

        self.app_benchmark_record = AppBenchmarkRecord(run_id, start_time, comment)
        self.run_id = run_id
    
    def create_state_record(self, uuid: uuid.UUID, state_name: str, state_type: str)-> int:
        insert_sql = 'INSERT INTO States (uuid, name, type, run_id) VALUES (?, ?, ?, ?)'
        record = (str(uuid), state_name, state_type, self.run_id)
        cursor = self.conn.cursor()
        cursor.execute(insert_sql, record)
        state_id = cursor.lastrowid
        self.conn.commit()
        return state_id
    
    def add_processing_record(self, record: ProcessingRecord):
        self._processing_records.append(record)
        if len(self._processing_records) >= self.flush_size:
            self.flush_processing_records()
    
    def add_input_record(self, record: InputRecord):
        self._input_records.append(record)
        if len(self._input_records) >= self.flush_size:
            self.flush_input_records()
    
    def run_query(self, sql):
        cursor = self.conn.cursor()
        cursor.execute(sql)
        result = list(cursor.fetchall())
        return result
    
    def flush_processing_records(self):
        insert_sql = f'''
        INSERT INTO ProcessingRecords
        (t, state_id, message_type, delta_t, run_id)
        VALUES (?, ?, ?, ?, {self.run_id})
        '''
        # ProcessingRecord type has corresponding fields...
        #['timestamp', state_name, state_type, message_type, delta_t_ns, run_id]
        self._flush_data(insert_sql, self._processing_records)
        rospy.loginfo("Performance Analysis - Flushed processing records")

    def flush_input_records(self):
        insert_sql = f'''
        INSERT INTO InputRecords
        (t, state_id, message_type, delta_t, run_id)
        VALUES (?, ?, ?, ?, {self.run_id})
        '''
        # The InputRecord type has corresponding fields...
        # [timestamp, state_name, state_type, message_type, delta_t_ns, run_id]
        self._flush_data(insert_sql, self._input_records)
        rospy.loginfo("Performance Analysis - Flushed input records")
    
    def _flush_data(self, sql, data):
        cursor = self.conn.cursor()
        cursor.executemany(sql, data)
        self.conn.commit()
        data.clear()

    def save(self):
        """Save all the data that we've collected so far to the database.
        """
        self.flush_input_records()
        self.flush_processing_records()


class DatabaseManager:

    _STOP = "stop"
    _INPUT_RECORD = "input_record"
    _PROCESSING_RECORD = "processing_record"
    _STATE_RECORD = "state_record"
    _RUN_QUERY = "run_query"
    _FLUSH_DATA = "flush_data"

    def __init__(self, start_time: int, comment: str, db_file: Path):
        self.start_time = start_time
        self.comment = comment
        self.db_file = db_file

        self.command_queue = queue.Queue()
        self.db_thread = threading.Thread(target=self.run)
        self.is_started = threading.Event()

        self._exception = None
        # this is to protect access to the exception field
        self._lock = threading.RLock()
    
    @property
    def exception(self):
        with self._lock:
            return self._exception
    
    def _put_cmd(self, cmd):
        self.command_queue.put(cmd, timeout=2)
        
    def start(self):
        self.db_thread.start()
        self.is_started.wait(timeout=2)

    def stop(self):
        cmd = (DatabaseManager._STOP, None)
        self._put_cmd(cmd)
        self.db_thread.join()
        if self.exception:
            raise self.exception

    def save_input_record(self, record: InputRecord):
        cmd = (DatabaseManager._INPUT_RECORD, record)
        self._put_cmd(cmd)

    def save_processing_record(self, record: ProcessingRecord):
        cmd = (DatabaseManager._PROCESSING_RECORD, record)
        self._put_cmd(cmd)
    
    def create_state_record(self, uuid: uuid.UUID, state_name: str, state_type: str) -> int:
        return_queue = queue.Queue()
        cmd = (DatabaseManager._STATE_RECORD, (uuid, state_name, state_type, return_queue))
        self._put_cmd(cmd)
        return return_queue.get(timeout=2) # if it takes more time, then something is wrong?
    
    def debug_run_query(self, sql: str, timeout=2):
        """
        Runs the provided query immediately for debugging purposes.

        Please note we don't flush the data to the database before running the query.
        So all cached records will not appear in the query results.

        See: debug_flush() if you want to flush the data to the database before running the query.

        Args:
            sql: the query to run
            timeout: how long to wait for the query to finish
                before raising an exception, must be None or a posative number
        """
        return_queue = queue.Queue()
        cmd = (DatabaseManager._RUN_QUERY, (sql, return_queue))
        self._put_cmd(cmd)
        return return_queue.get(timeout=timeout)
    
    def debug_flush(self):
        """
        Forces the system to flush all cached data out to the database immediately.
        This is primarily used for testing and debugging purposes.
        """
        cmd = (DatabaseManager._FLUSH_DATA, None)
        self._put_cmd(cmd)

    def run(self):
        try:
            self._run_internal()
        except Exception as e:
            with self._lock:
                self._exception = e
        
    def _run_internal(self):
        conn = None
        try:
            rospy.loginfo(f"Performance Analysis - Writing data to {self.db_file}")

            conn = sqlite3.connect(self.db_file)

            db = PerfDataStore(conn)
            db.create_tables()
            db.create_app_benchmark_record(self.start_time, self.comment)
            
            rospy.loginfo(f"Performance Analysis - Saving to '{self.db_file}'")
            rospy.loginfo(f"Performance Analysis - run_id = {db.run_id}")
            self.is_started.set()
            while True:
                # rospy.loginfo(f"Performance Analysis - Waiting for command")
                command, data = self.command_queue.get()
                # rospy.loginfo(f"Performance Analysis - Received command: {command}")

                if command == DatabaseManager._STOP:
                    db.save()
                    return
                elif command == DatabaseManager._PROCESSING_RECORD:
                    db.add_processing_record(data)
                elif command == DatabaseManager._INPUT_RECORD:
                    db.add_input_record(data)
                elif command == DatabaseManager._STATE_RECORD:
                    uuid, state_name, state_type, return_queue = data
                    state_id = db.create_state_record(uuid, state_name, state_type)
                    return_queue.put(state_id)
                elif command == DatabaseManager._RUN_QUERY:
                    sql, return_queue = data
                    print(sql)
                    result = db.run_query(sql)
                    return_queue.put(result)
                elif command == DatabaseManager._FLUSH_DATA:
                    db.save()
                else:
                    rospy.logerr(f"Performance Analysis - Unknown command: {command}")
                    raise ValueError(f"Unknown command: {command}")
        finally:
            rospy.loginfo(f"Performance Analysis - Closing database connection")
            if conn:
                conn.close()
            rospy.loginfo(f"Performance Analysis - Closed output file: '{self.db_file}'")

            


_database: DatabaseManager = None


def setup_benchmarking(start_time: int, comment: str, db_file=Path('perf_data.db')):
    global _database
    _database = DatabaseManager(start_time, comment, db_file)
    rospy.loginfo(f"Performance Analysis - Starting...")
    _database.start()


def stop_benchmarking():
    global _database
    rospy.logdebug(f"Performance Analysis - Stopping...")
    _database.stop()
    _database = None