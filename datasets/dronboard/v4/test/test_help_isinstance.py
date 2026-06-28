
import unittest
from typing import Union
from unittest.mock import patch

class Foo:
    pass

def my_func(arg1: Union[Foo,str]):
    if isinstance(arg1, Foo):
        return True
    elif isinstance(arg1, str):
        return False


MockFoo = type('MockFoo', (Foo,), {})

class TestPractice(unittest.TestCase):

    # Patch Foo with an actual subclass type so isinstance(..., Foo) still works
    @patch(__name__ + '.Foo', new=MockFoo)
    def test_my_func(self):
        self.assertFalse(my_func("test"))
        myFoo = MockFoo()
        self.assertTrue(my_func(myFoo))