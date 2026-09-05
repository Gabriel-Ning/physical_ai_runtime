"""Unit tests for the small application-output metadata envelope."""

import unittest
from types import SimpleNamespace

from isaacteleop_toolbox.runtime import _capture_output_metadata


class RuntimeMetadataTest(unittest.TestCase):
    def test_capture_output_metadata(self):
        session = SimpleNamespace(
            last_context=SimpleNamespace(
                execution_events=SimpleNamespace(
                    execution_state=SimpleNamespace(name="RUNNING")
                )
            ),
            last_step_info=SimpleNamespace(
                returned_frame_id=8,
                submitted_frame_id=9,
                returned_age_frames=1,
            ),
        )

        metadata = _capture_output_metadata(session, output_seq=12)

        self.assertEqual(
            metadata.as_dict(),
            {
                "output_seq": 12,
                "returned_frame_id": 8,
                "submitted_frame_id": 9,
                "returned_age_frames": 1,
                "execution_state": "running",
            },
        )


if __name__ == "__main__":
    unittest.main()
