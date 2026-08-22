import unittest
from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "foam_grasp" / "foam_segmentation_node.py").read_text()


class SegmentationCheckpointTest(unittest.TestCase):
    def test_checkpoint_loader_uses_weights_only(self):
        self.assertIn("weights_only=True", SOURCE)
        self.assertNotIn("weights_only=False", SOURCE)
