"""Release metadata and public status-version smoke tests."""

import unittest

import isaacteleop_toolbox
from isaacteleop_toolbox.ros_publishers import STATUS_SCHEMA_VERSION


class ReleaseContractTest(unittest.TestCase):
    def test_package_and_status_versions_are_explicit(self):
        self.assertEqual(isaacteleop_toolbox.__version__, "0.1.0")
        self.assertEqual(STATUS_SCHEMA_VERSION, 1)


if __name__ == "__main__":
    unittest.main()
