"""Score for the Avis film, on the edit's own timeline (film/render.mjs writes it; every time below comes from it).

A quiet bed the whole way (D major colour, 76 bpm, no drums until the flight), a low motor under the drive
sequence, air under every flight shot, one soft thud as each studio sequence opens, and a single chime when the
gate card comes up. Accent, never narrate.

    python vendor/teardown/run_sfx.py sims/avis-shared/score.py --timeline out/film/preview-timeline.json --out x.wav
"""

from sfx.score_kit import airy, bed, chords, hz, motor, thud
from sfx.synth import chime

TIMELINE = "../../out/film/avis-timeline.json"
OUT = "../../out/film/avis-score.wav"
TARGET_LUFS = -16.0
SEED = 23

CHORDS = chords("Dmaj9:D2 A2 C#3 E3 F#3/D1", "F#m7:F#2 C#3 E3 A3/F#1", "Gmaj9:G2 D3 F#3 A3 B3/G1", "A6sus:A2 D3 E3 F#3 B3/A1")
MOTIF = [(0.5, hz("F#5")), (2.0, hz("A5")), (3.0, hz("E5"))]


def score(mix, tl):
    dur = tl["duration"]
    segs = tl["segments"]
    left, right, low, drums = bed(dur, 76, CHORDS, MOTIF, motif_voice="epiano", kicks=(0,), snares=(), hats=0, swing=0.0, keys_vel=0.55, keys_lp=3200)
    mix.put_stereo(0, left, right, 0.11)
    mix.put(0, low, 0.10)
    for s in segs:
        t0, d = s["start"], s["dur"]
        if s["kind"] == "td":
            if s.get("seq") not in ("title",):
                mix.put(t0 + 0.05, thud(70, 0.06, 0.5), 0.05)
            if s.get("seq") == "drive":
                mix.put(t0 + 0.3, motor(max(0.5, d - 0.6), 120, 900, 9), 0.012)
            if s.get("seq") == "gates":
                mix.put(t0 + 0.4, chime([hz("A5"), hz("E6")], 2.4, 0.9), 0.035)
        elif s["kind"] == "bl":
            mix.put(t0, airy(d, 260, 900, 0.8), 0.022)
