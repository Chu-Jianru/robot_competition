"""Offline checks for the block stabilisation layer (no camera, no policy)."""
import unittest

from block_tracks import BlockStabilizer, StabilizerConfig


def block(x, y=0.0, hx=0.010, hy=0.010, own=True):
    return (x, y, hx, hy, own)


class MergeTest(unittest.TestCase):
    def test_over_split_halves_become_one_block(self):
        stabilizer = BlockStabilizer(StabilizerConfig(confirm_hits=1))
        result = stabilizer.update([block(0.000), block(0.010)])
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0][0], 0.005, delta=0.003)

    def test_touching_cubes_stay_two(self):
        stabilizer = BlockStabilizer(StabilizerConfig(confirm_hits=1))
        result = stabilizer.update([block(0.0), block(0.020)])
        self.assertEqual(len(result), 2)


class ConfirmationTest(unittest.TestCase):
    def test_single_frame_ghost_is_not_reported(self):
        stabilizer = BlockStabilizer()
        stabilizer.update([block(0.0)])                      # cold start accepts
        stabilizer.update([block(0.0), block(0.3)])          # ghost appears once
        self.assertEqual(len(stabilizer.update([block(0.0)])), 1)

    def test_cold_start_reports_the_first_frame(self):
        stabilizer = BlockStabilizer()
        self.assertEqual(len(stabilizer.update([block(0.0), block(0.2)])), 2)

    def test_second_frame_confirms_the_block(self):
        stabilizer = BlockStabilizer()
        stabilizer.update([block(0.0)])
        result = stabilizer.update([block(0.001)])
        self.assertEqual(len(result), 1)


class GapTest(unittest.TestCase):
    def setUp(self):
        self.stabilizer = BlockStabilizer()
        for _ in range(3):
            self.stabilizer.update([block(0.0)])

    def test_short_gap_keeps_the_block(self):
        for _ in range(3):
            self.assertEqual(len(self.stabilizer.update([])), 1)

    def test_long_gap_drops_the_block(self):
        for _ in range(4):
            self.stabilizer.update([])
        self.assertEqual(self.stabilizer.update([]), [])

    def test_block_is_removed_when_only_the_ghost_returns(self):
        for _ in range(4):
            self.stabilizer.update([])
        self.assertEqual(self.stabilizer.update([block(0.5)]), [])


class SmoothingTest(unittest.TestCase):
    def test_reported_position_moves_less_than_raw_input(self):
        stabilizer = BlockStabilizer(StabilizerConfig(confirm_hits=1))
        raw = [0.0, 0.010, 0.0, 0.010, 0.0, 0.010]
        reported = [stabilizer.update([block(x)])[0][0] for x in raw[1:]]
        self.assertLess(max(reported) - min(reported), max(raw) - min(raw))

    def test_ownership_follows_the_majority(self):
        stabilizer = BlockStabilizer(StabilizerConfig(confirm_hits=1))
        for own in (True, True, True, False):
            result = stabilizer.update([block(0.0, own=own)])
        self.assertTrue(result[0][4])


class RobustnessTest(unittest.TestCase):
    def test_empty_and_malformed_input_do_not_crash(self):
        stabilizer = BlockStabilizer()
        self.assertEqual(stabilizer.update([]), [])
        self.assertEqual(stabilizer.update(()), [])
        self.assertIn("raw", stabilizer.stats)

    def test_two_separate_blocks_are_tracked_independently(self):
        stabilizer = BlockStabilizer(StabilizerConfig(confirm_hits=1))
        for _ in range(3):
            result = stabilizer.update([block(-0.2), block(0.2)])
        self.assertEqual(len(result), 2)
        # Removing one keeps the other.
        for _ in range(2):
            result = stabilizer.update([block(-0.2)])
        self.assertEqual(len(result), 2)   # the missing one is still bridged
        self.assertAlmostEqual(sorted(b[0] for b in result)[0], -0.2, delta=0.01)


if __name__ == "__main__":
    unittest.main()
