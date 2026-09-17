"""detection_source.py — the camera detections the planner is allowed to see.

Two live streams are read here, and neither is started here:

* the gaze (``eyes_gaze.EyeGazer``), which already runs the shared detector at
  ``cam_hz`` while the eyes are on.  It is read first because it is the pass the
  robot pays for anyway.  Before this the planner read only
  ``cvf.last_detections``, which the CV overlay refreshes while it happens to be
  in one of its object modes — so live, the camera saw a surfboard and the map
  held no objects at all;
* ``cvf.last_detections``, the overlay's own hits, kept behind the gaze's and
  still authoritative when a name appears in both.

Nothing is de-duplicated here, because there is nothing to de-duplicate:
``SpatialMemory.observe_object`` already folds a second sighting of the same
name, bearing and range into one object, so two paths reporting one chair stay
one chair.

The gaze publishes its boxes as a fraction of the frame; the memory's
``box_bearing_deg`` takes pixels.  Scaling by the constants in ``perception`` is
exact for a bearing — a bearing is a difference of fractions, so the scale
cancels — and one published unit beats publishing the same box twice.

The open-vocabulary top-up is unchanged: ``detect_world()`` — the expensive
open-vocabulary pass Lance speaks from — is only run while a *named* target is
missing from the live stream, and at most once every ``world_scan_s``.
"""

from perception import FRAME_HEIGHT_PX, FRAME_WIDTH_PX, names_match

# A gaze that has stopped stepping stops feeding the planner: its last pass is
# not news any more, and re-fusing it would let an object that moved away keep
# refusing the heading it was seen at.  Generous next to the gaze's own rate,
# because one camera pass costs ~1.3 s under load.
GAZE_MAX_AGE_S = 5.0


class DetectionSource:
    """Live detections, topped up with a throttled open-vocabulary scan."""

    def __init__(self, cvf, world_scan_s=1.0, gaze=None):
        self._cvf = cvf
        self.gaze = gaze
        self.world_scan_s = world_scan_s
        self.gaze_max_age_s = GAZE_MAX_AGE_S
        self._world_dets = []
        self._world_scan_t = 0.0
        self._clock = None      # injectable for tests; defaults to time.time

    # ── reads ─────────────────────────────────────────────────────────────
    def live(self):
        """Every live camera hit: the gaze's pass, then the overlay's."""
        return self.from_gaze() + self.overlay()

    def overlay(self):
        """Whatever the CV overlay's frame loop last published."""
        return list(getattr(self._cvf, 'last_detections', None) or [])

    def from_gaze(self):
        """The gaze's last published pass, in the memory's pixel-box terms.

        Empty unless the gaze is running and its camera is answering, so an
        operator who switches the eyes off leaves the planner exactly as it was.
        A gaze whose loop has stopped stepping is treated as silent rather than
        fresh — otherwise its boxes would be fused again ten times a second for
        as long as the app runs.
        """
        gaze = self.gaze
        if gaze is None:
            return []
        try:
            status = gaze.status()
        except Exception as e:                                   # noqa: BLE001
            print(f"[detections] gaze status failed: {e}")
            return []
        if not status.get('enabled') or status.get('frame_stale'):
            return []
        age = status.get('age_s')
        if age is not None and age > self.gaze_max_age_s:
            return []
        out = []
        for hit in status.get('detections') or []:
            box = hit.get('box')
            if not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            x1, y1, x2, y2 = (float(v) for v in box)
            out.append({'name': str(hit.get('name') or 'object'),
                        'confidence': max(0.0, float(hit.get('conf') or 0.0)),
                        'box': [int(round(x1 * FRAME_WIDTH_PX)),
                                int(round(y1 * FRAME_HEIGHT_PX)),
                                int(round(x2 * FRAME_WIDTH_PX)),
                                int(round(y2 * FRAME_HEIGHT_PX))]})
        return out

    def for_target(self, name, now=None):
        """Detections for pursuing `name` (None -> just the live streams).

        When the live streams have no match, the open-vocabulary pass is
        consulted (throttled) and its hits are appended.
        """
        dets = self.live()
        if not name:
            self._world_dets = []
            return dets
        if any(names_match(d.get('name', ''), name) for d in dets):
            return dets
        now = self._now(now)
        world = getattr(self._cvf, 'detect_world', None)
        if callable(world) and now - self._world_scan_t >= self.world_scan_s:
            self._world_scan_t = now
            try:
                self._world_dets = list(world() or [])
            except Exception as e:
                print(f"[detections] open-vocabulary scan failed: {e}")
                self._world_dets = []
        if self._world_dets:
            seen = {d.get('name') for d in dets}
            dets = dets + [d for d in self._world_dets if d.get('name') not in seen]
        return dets

    # ── internals ─────────────────────────────────────────────────────────
    def _now(self, now):
        if now is not None:
            return now
        if self._clock is None:
            import time
            self._clock = time.time
        return self._clock()
