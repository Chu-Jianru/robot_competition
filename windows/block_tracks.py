"""Cross-frame stabilisation for red-block detections (Windows live path).

The per-frame detector is deliberately stateless: one frame's split of a pile is
`round(area / instance_area)` with `instance_area` re-estimated from that same
frame, so the same pile can come back as 17 or 19 blocks, a single frame can
invent a split, and block centres move several millimetres between frames.
`competition_policy` already keeps unmatched blocks for 0.45 s, but it accepts
every new detection immediately and never smooths positions.

This layer adds the three things that decide whether the block list looks stable:

1. **Minimum separation merge** - two 20 mm cubes cannot have centres closer
   than 20 mm, so detections closer than 12 mm are one over-split block, merged
   deterministically without any temporal lag.
2. **Confirmation** - a new block is only reported after `confirm_hits` frames,
   so a one-frame ghost never reaches the policy.
3. **Position smoothing** - an exponential average per track, so planning sees a
   steady centre instead of the raw per-frame jitter.

Pure Python: no camera, no ROS, no policy import. Input and output are the same
`(x, y, half_x, half_y, own)` tuples `run_task.observe` already produces.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

Block = tuple


@dataclass
class StabilizerConfig:
    merge_m: float = 0.012          # centres closer than this are one block
    gate_m: float = 0.030           # association gate between frames
    confirm_hits: int = 2           # frames a new block must appear in
    drop_misses: int = 3            # frames a block may be missing before removal
    position_alpha: float = 0.45    # weight of the new measurement
    own_alpha: float = 0.34         # weight of the new own/enemy vote


@dataclass
class _Track:
    x: float
    y: float
    hx: float
    hy: float
    own: float                      # running vote in [0, 1], >= 0.5 means own
    hits: int = 1
    misses: int = 0

    def as_block(self) -> Block:
        return (self.x, self.y, self.hx, self.hy, self.own >= 0.5)


class BlockStabilizer:
    """Merge, confirm and smooth block detections across frames."""

    def __init__(self, config: StabilizerConfig | None = None) -> None:
        self.config = config or StabilizerConfig()
        self.tracks: list[_Track] = []
        self.stats: dict = {}
        self._cold = True

    def reset(self) -> None:
        self.tracks = []
        self._cold = True

    def update(self, blocks, now: float | None = None) -> list[Block]:
        config = self.config
        merged = _merge(blocks, config.merge_m)
        # At a cold start there is nothing to be sceptical about, so the first
        # frame is accepted as-is instead of losing one frame to confirmation.
        cold = self._cold and bool(merged)
        self._cold = False
        used: set[int] = set()
        for x, y, hx, hy, own in merged:
            best, best_index = None, None
            for index, track in enumerate(self.tracks):
                if index in used:
                    continue
                distance = math.hypot(track.x - x, track.y - y)
                if distance <= config.gate_m and (best is None or distance < best):
                    best, best_index = distance, index
            if best_index is None:
                self.tracks.append(_Track(x, y, hx, hy, 1.0 if own else 0.0,
                                          hits=config.confirm_hits if cold else 1))
                used.add(len(self.tracks) - 1)
                continue
            used.add(best_index)
            track = self.tracks[best_index]
            track.x += config.position_alpha * (x - track.x)
            track.y += config.position_alpha * (y - track.y)
            track.hx = max(track.hx, hx)
            track.hy = max(track.hy, hy)
            track.own += config.own_alpha * ((1.0 if own else 0.0) - track.own)
            track.hits += 1
            track.misses = 0
        survivors = []
        for index, track in enumerate(self.tracks):
            if index not in used:
                track.misses += 1
            # A confirmed track survives a short gap; a tentative one does not
            # get extra time, otherwise a ghost would linger for drop_misses.
            if track.misses > config.drop_misses or (
                    track.hits < config.confirm_hits and track.misses):
                continue
            survivors.append(track)
        self.tracks = survivors
        # A confirmed track keeps being reported through a short gap, otherwise
        # one missed frame would empty the list the policy plans with.
        reported = [track.as_block() for track in self.tracks
                    if track.hits >= config.confirm_hits]
        self.stats = {"raw": len(blocks), "merged": len(merged),
                      "tracks": len(self.tracks), "reported": len(reported)}
        return sorted(reported, key=lambda block: (block[0], block[1]))


def _merge(blocks, distance: float) -> list[Block]:
    """Greedy merge of detections closer than `distance`, largest first."""
    ordered = sorted(blocks, key=lambda block: -(abs(block[2]) * abs(block[3])))
    clusters: list[dict] = []
    for x, y, hx, hy, own in ordered:
        for cluster in clusters:
            if math.hypot(cluster["x"] - x, cluster["y"] - y) < distance:
                cluster["x"] += 0.5 * (x - cluster["x"])
                cluster["y"] += 0.5 * (y - cluster["y"])
                cluster["hx"] = max(cluster["hx"], abs(hx))
                cluster["hy"] = max(cluster["hy"], abs(hy))
                cluster["votes"] += 1.0 if own else 0.0
                cluster["seen"] += 1
                break
        else:
            clusters.append({"x": x, "y": y, "hx": abs(hx), "hy": abs(hy),
                             "votes": 1.0 if own else 0.0, "seen": 1})
    return [(cluster["x"], cluster["y"], cluster["hx"], cluster["hy"],
             cluster["votes"] >= cluster["seen"] / 2.0) for cluster in clusters]
