import sys
import time
import unittest
import uuid

import dr_onboard_autonomy.states.collections.performance_analysis as perf_analysis
from dr_onboard_autonomy.states import BaseState

from . import mock_types

def run_query(db: perf_analysis.DatabaseManager, query: str):
    """Run the query and return the result. If there is an exception, raise it.

    We need this method because the db manager will raise an exception if there is an error,
    But this exception is raised in a different thread, so we can't catch it.
    Instead we'll likely see a queue.Empty exception.

    The queue.Empty exception isn't that helpful. The real error is in the
    db.exception attribute. The only reason our queue is empty is because the 
    database thread crashed. So let's raise the exception that caused the error.
    
    Args:
        db (perf_analysis.DatabaseManager): The database manager
        query (str): The query to run
    
    Returns whatever your query returns. If your query returns some records,
    it will be a list of tuples.
    """
    try:
        return db.debug_run_query(query, timeout=1.5)
    except Exception as e:
        if db.exception is not None:
            raise db.exception
        else:
            raise e

class TestPerformanceAnalysis(unittest.TestCase):

    def setUp(self):
        # AppBenchmarkRecord fields:
        self.start = time.perf_counter_ns()
        self.comment = "UNIT TEST"
        self.db_file = ":memory:"

        # create the database manager
        self.db = perf_analysis.DatabaseManager(self.start, self.comment, self.db_file)

    def tearDown(self):
        self.db.stop()

    def test_everything(self):
        # AppBenchmarkRecord fields:
        start = self.start
        comment = self.comment

        # the DataBaseManager
        db = self.db
        
        db.start()
    
        # we will call the `create_state_record` method 3 times
        state_ids = []
        for i in range(3):
            # create_state_record

            state_uuid = uuid.uuid4()
            state_name: str = f"test_state {i}"
            state_type: str = "BaseState"
            state_id_last = db.create_state_record(state_uuid, state_name, state_type)
            state_ids.append(state_id_last)
        
        # test to make sure all state_ids are unique
        self.assertEqual(len(state_ids), len(set(state_ids)))

        kwargs = mock_types.mock_state_kwargs({
            "performance_analysis": True,
            "performance_analysis_database": db,
            "name": "test_state @@@@",
            "trjectory_class": None,
            "outcomes": ["success", "failure"]
        })
        state = BaseState(**kwargs)
        perf_queue = state.message_queue

        self.assertIsInstance(perf_queue, perf_analysis.PerformanceAnalysisQueue)

        # Lets add two message to the queue and get them out to make sure we are recording metrics
        home = kwargs["home"]
        msg1 = mock_types.mock_position_message(home["latitude"], home["longitude"], home["altitude"])
        perf_queue.put(msg1)
        out_msg = perf_queue.get()
        self.assertEqual(msg1, out_msg)

        msg2 = mock_types.mock_position_message(home["latitude"], home["longitude"], 0.0)
        perf_queue.put(msg2)
        out_msg2 = perf_queue.get()
        self.assertEqual(msg2, out_msg2)

        # last we call the finish_processing method
        # this will create a processing record for our 2nd message
        perf_queue.finish_processing() 

        # now we will use the run_queue method to make sure 
        db.debug_flush()

        # check the app record
        app_query = "SELECT * FROM AppBenchmarkRecords"
        x = run_query(db, app_query)
        self.assertEqual(len(x), 1)
        # x should have a tuple with the AppBenchmarkRecord
        run_id, app_uuid, start_time, comment = x[0]
        self.assertEqual(run_id, 1)
        # make sure app_uuid is a valid uuid
        app_uuid = uuid.UUID(app_uuid)
        self.assertIsInstance(app_uuid, uuid.UUID)
        self.assertEqual(start_time, start)
        self.assertEqual(comment, comment)

        # check the state record
        state_query = "SELECT * FROM States"
        states_records = run_query(db, state_query)
        self.assertEqual(len(states_records), 4)

        # make sure the app_id and state_id are correct for the original 3
        # states we created above (note we created a 4th state when we created
        # that BaseState instance we will check that after)
        for i, state_id_last in enumerate(state_ids):
            actual_id, state_uuid, state_name, state_type, app_id = states_records[i]
            self.assertEqual(actual_id, state_id_last)
            self.assertEqual(state_name, f"test_state {i}")
            self.assertEqual(state_type, "BaseState")
            self.assertEqual(app_id, 1)
            # make sure we have a valid uuid
            state_uuid = uuid.UUID(state_uuid)
            self.assertIsInstance(state_uuid, uuid.UUID)
        # now lets check the 4th state
        state_id_last, state_uuid, state_name, state_type, app_id = states_records[3]
        self.assertEqual(state_name, kwargs["name"])
        self.assertEqual(state_type, state.__class__.__name__)
        self.assertEqual(app_id, run_id)
        # make sure we have a valid uuid
        state_uuid = uuid.UUID(state_uuid)
        self.assertIsInstance(state_uuid, uuid.UUID)
        # make sure the state_id is unique
        self.assertNotIn(state_id_last, state_ids)

        # check the input records
        input_query = "SELECT * FROM InputRecords"
        input_records = run_query(db, input_query)
        self.assertEqual(len(input_records), 2)
        # verify the fields
        # t INTEGER, state_id INTEGER, message_type TEXT, delta_t INTEGER, run_id INTEGER,
        for i, (t, state_id, message_type, delta_t, run_id) in enumerate(input_records):
            # t must be less than time.perf_counter_ns()
            self.assertLess(t, time.perf_counter_ns())
            # since we put this message in the queue that was made for the 4th state
            # we need to make sure the state_id is correct
            self.assertEqual(state_id, state_id_last)
            # the message type was "position" make sure that's right
            self.assertEqual(message_type, "position")
            if i == 0:
                # sinc we never put another message in the queue before the first one
                # delta_t should be None
                self.assertIsNone(delta_t)
            else:
                # Make sure delta_t is greater than 0
                # this measures the time difference between when the first message was put in the queue
                # and the second message was put in the queue. It must be greater than 0 nano seconds
                # to pass the test... if we run this on a fast enough computer, this might fail
                # but that's unlikely to happen anytime soon
                self.assertGreater(delta_t, 0)
            # make sure the run_id matches the app_id we queried above
            self.assertEqual(run_id, run_id)

        # check the processing records
        processing_query = "SELECT * FROM ProcessingRecords"
        processing_records = run_query(db, processing_query)
        self.assertEqual(len(processing_records), 2)
        # verify the fields
        # t INTEGER, state_id INTEGER, message_type TEXT, delta_t INTEGER, run_id INTEGER,
        for i, (t, actual_state_id, msg_type, delta_t, run_id) in enumerate(processing_records):
            # make sure t is an int
            self.assertIsInstance(t, int)
            # make sure t is less than time.perf_counter_ns()
            self.assertLess(t, time.perf_counter_ns())
            # make sure the state_id is correct. It should match the last one,
            # since this message_queue was created for the 4th state
            self.assertEqual(actual_state_id, state_id_last)
            # the message type should be "position" since that's the only
            # type of message we put in the queue
            self.assertEqual(msg_type, "position")
            # delta_t should be greater than 0 every time.
            # the the delta_t value is hte time difference between get() calls
            # or for the last message it's the time difference between the
            # previous get() call and the finish_processing() call
            self.assertGreater(delta_t, 0)
            # make sure the run_id matches the app_id we queried above
            self.assertEqual(run_id, run_id)
        db.stop()
        # ok that's it for the test. Make sure we didn't get any exceptions
        # in the database thread
        self.assertIsNone(db.exception)