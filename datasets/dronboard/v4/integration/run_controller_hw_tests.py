import unittest

test_loader = unittest.TestLoader()
test_runner = unittest.TextTestRunner()
test_runner.run(test_loader.discover("./integration", pattern="*_controller_test.py"))