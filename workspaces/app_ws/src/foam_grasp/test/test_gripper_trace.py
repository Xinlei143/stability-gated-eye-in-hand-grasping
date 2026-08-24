import unittest

from foam_grasp.gripper_trace import make_trace_row


class GripperTraceTest(unittest.TestCase):
    def test_row_records_commands_feedback_stamps_and_symmetry(self):
        row = make_trace_row(
            wall_time_s=12.5,
            joint7_command=0.020,
            joint8_command=-0.020,
            joint7_feedback=0.026,
            joint8_feedback=-0.0214,
            joint7_stamp_s=101.02,
            joint8_stamp_s=101.01,
        )

        self.assertEqual(row["wall_time_s"], 12.5)
        self.assertEqual(row["joint7_command"], 0.020)
        self.assertEqual(row["joint8_command"], -0.020)
        self.assertEqual(row["joint7_feedback"], 0.026)
        self.assertEqual(row["joint8_feedback"], -0.0214)
        self.assertEqual(row["joint7_stamp_s"], 101.02)
        self.assertEqual(row["joint8_stamp_s"], 101.01)
        self.assertAlmostEqual(row["symmetry_error_m"], 0.0046)

    def test_row_leaves_missing_feedback_and_symmetry_empty(self):
        row = make_trace_row(
            wall_time_s=1.0,
            joint7_command=0.020,
            joint8_command=-0.020,
            joint7_feedback=None,
            joint8_feedback=-0.020,
            joint7_stamp_s=None,
            joint8_stamp_s=2.0,
        )

        self.assertIsNone(row["joint7_feedback"])
        self.assertIsNone(row["joint7_stamp_s"])
        self.assertIsNone(row["symmetry_error_m"])


if __name__ == "__main__":
    unittest.main()
