"""film/bird_px.py puts the marker where env/test_shots.py's independent pinhole sees the bird, on the frames
film/render.mjs renders for the edit's wide shot, and the bird there is too small to read without it.

    out/env/.venv/bin/python -I -m unittest discover -s film/test -p "test_*.py"
"""
import io
import json
import os
import sys
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "env"))
import numpy as np  # noqa: E402

import bird_px  # noqa: E402
import test_shots as T  # noqa: E402


class WideMarker(unittest.TestCase):
    def test_marker_follows_the_bird(self):
        seg = next(s for s in T.SEGS if s.get("marker"))
        fps, (a, b) = T.plate(seg)
        buf = io.StringIO()
        with redirect_stdout(buf):
            bird_px.main(["scenario=%s" % seg["scenario"], "shot=%s" % seg["shot"], "fps=%s" % T.EDIT["fps"],
                          "slow=%s" % seg.get("slow", 1), "i0=%d" % a, "n=%d" % (b - a)] + (["lens=%s" % seg["lens"]] if seg.get("lens") else []))
        got = json.loads(buf.getvalue())
        self.assertEqual(len(got["px"]), b - a)
        fl = T.flight(seg["scenario"], fps)
        sh = T.shot(fl, seg["shot"], (a, b))
        lens = seg.get("lens") or sh["lens"]
        self.assertEqual(got["lens"], lens)
        for k, i in enumerate(range(a, b)):
            x, y, d = T.pixel_xy(sh["cam"][i], sh["target"][i], lens, fl["pos"][i][None])
            self.assertLess(abs(got["px"][k][0] - x[0]), 0.2, "frame %d" % i)
            self.assertLess(abs(got["px"][k][1] - y[0]), 0.2, "frame %d" % i)
            self.assertLess(abs(got["px"][k][2] - d[0]), 0.2, "frame %d" % i)
        # what the marker is for: a bird a few pixels across, on screen
        self.assertLess(max(got["span_px"]), 12.0)
        self.assertGreater(min(got["span_px"]), 0.5)
        px = np.array(got["px"])
        self.assertTrue(((px[:, 0] > 0) & (px[:, 0] < 1920) & (px[:, 1] > 0) & (px[:, 1] < 1080)).all())


if __name__ == "__main__":
    unittest.main()
