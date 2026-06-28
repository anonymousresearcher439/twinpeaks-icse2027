'''
looks like rospy logging builds upon python logging.
the following disables all but critical logs while running unit tests
'''
import logging
logging.disable(logging.CRITICAL)