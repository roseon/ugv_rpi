#!/usr/bin/env python3
"""Headless tests for speech_face.py — no robot, no speaker, no Azure.

Builds WAV bytes in memory, so the envelope maths, the chunk walking and the
timeline are covered without any audio device at all.

Run: python speech_face_selftest.py
"""

import math
import struct
import threading

from speech_face import (ATTACK_TAU, BREATH_AMP, BREATH_BASE, ENV_FPS, GRACE_S,
                         RELEASE_TAU, SMILE_IDLE, SMILE_SPEAKING, Speech, WINDOW_N,
                         envelope_from_text, envelope_from_wav)

CHECKS = 0
FAILURES = []


def check(label, cond, detail=""):
    global CHECKS
    CHECKS += 1
    if not cond:
        FAILURES.append(f"{label}  {detail}")
        print(f"FAIL  {label}  {detail}")
    else:
        print(f"PASS  {label}")


def make_wav(frames, rate=24000, channels=1, bits=16, audio_format=1, extra=b"",
             declared_data=None, riff=True):
    """frames: list of per-frame sample lists.  Returns WAV bytes."""
    if bits == 16:
        data = b"".join(struct.pack("<h", v) for f in frames for v in f)
    else:
        data = bytes((v + 128) & 0xFF for f in frames for v in f)
    block = channels * bits // 8
    size = len(data) if declared_data is None else declared_data
    body = (b"fmt " + struct.pack("<IHHIIHH", 16, audio_format, channels, rate,
                                  rate * block, block, bits)
            + extra
            + b"data" + struct.pack("<I", size) + data)
    return (b"RIFF" + struct.pack("<I", 4 + len(body)) + b"WAVE" + body) if riff \
        else body


def tone(secs, rate=24000, amp=12000, freq=440.0):
    return [[int(amp * math.sin(2 * math.pi * freq * i / rate))]
            for i in range(int(secs * rate))]


SILENT = [[0] for _ in range(4800)]


# ── measuring a WAV ───────────────────────────────────────────────────────────
loud = envelope_from_wav(make_wav(tone(0.20) + SILENT))[0]
wav = make_wav(tone(0.20) + SILENT)
levels, duration = envelope_from_wav(wav)
check("a WAV's duration comes from its samples", abs(duration - 0.40) < 0.01,
      f"got {duration}")

half = len(levels) // 2
check("the loud half of the audio opens the mouth",
      max(levels[:half]) > 0.95, f"peak {max(levels[:half]):.3f}")
# From one frame past the boundary: the 3-frame blur deliberately bridges it, so
# the first silent frame still carries a third of the syllable before it.
check("the silent half closes it", max(levels[half + 1:]) < 0.05,
      f"peak {max(levels[half:]):.3f}")
check("the envelope runs at display rate", abs(len(levels) - 0.40 * ENV_FPS) <= 2,
      f"got {len(levels)} frames")

half_amp = envelope_from_wav(make_wav(tone(0.10, amp=4000)))[0]
check("a quieter take still opens the mouth (normalised)",
      max(half_amp) > 0.95, f"peak {max(half_amp):.3f}")

quiet = make_wav(tone(0.10, amp=4000))
quiet_peak = max(envelope_from_wav(quiet)[0])
loud_peak = max(envelope_from_wav(make_wav(tone(0.10)))[0])
check("...so per-frame loudness is not what the mouth follows, shape is",
      abs(quiet_peak - loud_peak) < 1e-6, f"got {quiet_peak} vs {loud_peak}")

sil = envelope_from_wav(make_wav(SILENT))
check("silence is a closed mouth, not an error",
      sil[0] is not None and max(sil[0]) == 0.0, f"got {sil[0][:3]}")

stereo = make_wav([[12000, 0] for _ in range(2400)], channels=2)
s_levels, s_dur = envelope_from_wav(stereo)
check("a stereo WAV is averaged, not misread", s_levels and max(s_levels) > 0.95,
      f"got {max(s_levels) if s_levels else None}")

eight = make_wav([[100] * 2400], bits=8)
e_levels, _ = envelope_from_wav(eight)
check("an 8-bit WAV is read as unsigned PCM", e_levels and max(e_levels) > 0.9,
      f"got {max(e_levels) if e_levels else None}")

odd = b"LIST" + struct.pack("<I", 5) + b"abcde" + b"\x00"      # odd chunk, word-aligned
pad_levels, _ = envelope_from_wav(make_wav(tone(0.10), extra=odd))
check("the chunk walker survives an odd-sized chunk before the data",
      pad_levels and max(pad_levels) > 0.9, f"got {pad_levels is not None}")

lying = make_wav(tone(0.10), declared_data=10 ** 7)
l_levels, _ = envelope_from_wav(lying)
check("a data chunk that claims more bytes than exist is clamped",
      l_levels and max(l_levels) > 0.9, f"got {l_levels is not None}")

check("a compressed/float stream is refused rather than read as PCM",
      envelope_from_wav(make_wav(tone(0.1), audio_format=3)) == (None, None))
check("a truncated file is refused", envelope_from_wav(b"RIFF\x00\x00\x00\x00WAVE") == (None, None))
check("a bare data chunk with no fmt is refused",
      envelope_from_wav(make_wav(tone(0.1))[12:20] + b"\x00" * 40) == (None, None))
check("no bytes at all is refused", envelope_from_wav(b"") == (None, None))
check("bytes that are not a RIFF are refused",
      envelope_from_wav(b"not a wav" * 20) == (None, None))


# ── the text fallback ─────────────────────────────────────────────────────────
short = envelope_from_text("hi")
long_text = envelope_from_text("this is a considerably longer sentence to speak")
check("the text fallback scales its duration with the text",
      long_text[1] > short[1] * 2, f"got {short[1]} vs {long_text[1]}")
check("the text fallback stays inside 0..1",
      all(0.0 <= v <= 1.0 for v in long_text[0]))
vowel = envelope_from_text("aaaa")
cons = envelope_from_text("bbbb")
check("vowels open the text envelope wider than consonants",
      max(vowel[0]) > max(cons[0]), f"got {max(vowel[0]):.2f} vs {max(cons[0]):.2f}")
check("the text fallback is deterministic", envelope_from_text("hello") == envelope_from_text("hello"))


# ── the timeline ──────────────────────────────────────────────────────────────
class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


clock = Clock()
face = Speech(clock=clock)
st = face.snapshot()
check("nothing is speaking before anything speaks",
      st["speaking"] is False and st["text"] == "", f"got {st['speaking']}")
check("an idle mouth still breathes, and only a little",
      BREATH_BASE - BREATH_AMP <= st["open"][-1] <= BREATH_BASE + BREATH_AMP + 0.001,
      f"got {st['open'][-1]}")
check("the idle face sits on the same curve both faces read",
      st["wide"][-1] == 0.0 and abs(st["smile"][-1] - SMILE_IDLE) < 0.02,
      f"got {st['wide'][-1]}/{st['smile'][-1]}")

seq = face.begin("hello there", wav=make_wav(tone(0.5) + SILENT))
peak = 0.0
for i in range(24):                       # half a second of frames
    clock.t += 1.0 / 60.0
    peak = max(peak, face.snapshot()["open"][-1])
st = face.snapshot()
check("an utterance with audio reports itself as audio",
      st["speaking"] and st["source"] == "audio" and st["text"] == "hello there",
      f"got {st['source']}/{st['text']}")
check("the jaw opens on the audio, not on the wall clock", peak > 0.7,
      f"peak opening {peak:.3f}")
check("the smile flattens out while speaking",
      abs(st["smile"][-1] - SMILE_SPEAKING) < 0.03, f"got {st['smile'][-1]}")

clock.t += 0.4                            # into the silent tail
tail = face.snapshot()["open"][-1]
check("a silent tail closes the mouth before the utterance ends",
      tail < 0.25, f"got {tail}")

face.end(seq)
clock.t += 0.05
check("playback returning early does not cut the animation short",
      face.snapshot()["speaking"] is True, "the mouth stopped with the player")
clock.t += 0.5 + GRACE_S + 0.01
after = face.snapshot()
check("...and the utterance still ends on its own timeline",
      after["speaking"] is False and after["text"] == "", f"got {after['speaking']}")

clock.t += 1.0
face.begin("no wav at all")
st = face.snapshot()
check("an utterance with no WAV falls back to the text envelope",
      st["speaking"] and st["source"] == "text", f"got {st['source']}")

# A newer utterance must not be closed by the previous player returning.
clock.t += 0.01
old_seq = face._seq
clock.t += 1.0
new_seq = face.begin("second")
face.end(old_seq)
check("the previous player returning cannot close a newer utterance",
      face.snapshot()["speaking"] and face.snapshot()["text"] == "second",
      f"got {face.snapshot()['text']!r} speaking={face.snapshot()['speaking']}")
face.end(new_seq)

long_face = Speech(clock=Clock())
long_face.begin("x" * 400)
lst = long_face.snapshot()
check("a long utterance cannot bloat the status payload",
      len(lst["open"]) <= WINDOW_N, f"got {len(lst['open'])}")
check("the three curves travel together",
      len(lst["open"]) == len(lst["wide"]) == len(lst["smile"])
      and lst["sps"] == ENV_FPS, f"got {len(lst['open'])}/{len(lst['wide'])}/{len(lst['smile'])}")
check("status is JSON-shaped", isinstance(lst["open"][0], float)
      and isinstance(lst["sps"], int) and isinstance(lst["text"], str))


# ── the model: the one definition of how the mouth moves ──────────────────────
# These are the checks the panel used to make about its own copy of the model.
rush = Speech(clock=Clock(0.0))
rush.begin("x", wav=make_wav(tone(0.30) + SILENT))
opens = []
for i in range(90):
    rush._clock.t = i / 60.0
    opens.append(rush.snapshot()["open"][-1])
peak = max(opens)
rise = next(i for i, v in enumerate(opens) if v >= peak * 0.9)
fall = next(i for i, v in enumerate(opens[rise:], rise) if v <= peak * 0.1)
print(f"   jaw: to 90% in {rise} frames, back to 10% in {fall - rise}")
check("the jaw opens faster than it closes - most of what reads as speech",
      (fall - rise) > rise * 1.5, f"rise {rise}, fall {fall - rise}")
check("...and the asymmetry is the model's own constants",
      RELEASE_TAU > ATTACK_TAU * 2)

# Silence first, then a syllable: a width only rises when the level rises, so a
# take that starts loud has no attack to measure (it starts at its peak, and the
# corners can only narrow from there).
wides = []
syl = Speech(clock=Clock(0.0))
syl.begin("y", wav=make_wav(SILENT + tone(0.25) + SILENT))
for i in range(150):                       # the take, its grace tail, then idle
    syl._clock.t = i / 60.0
    wides.append(syl.snapshot()["wide"][-1])
print("   corner width over silence->syllable: %+.3f peak, %+.3f low"
      % (max(wides), min(wides)))
check("a syllable's attack widens the corners", max(wides) > 0.05,
      f"peak width {max(wides):.3f}")
check("the corners stay inside the model's clamp",
      min(wides) >= -0.351 and max(wides) <= 0.351,
      f"got {min(wides):.3f}..{max(wides):.3f}")
check("the corners come back to neutral once the utterance is over",
      abs(wides[-1]) < 0.05, f"got {wides[-1]:.3f}")

rest = Speech(clock=Clock(0.0))
rest.begin("loud from the first frame", wav=make_wav(tone(0.4)))
firsts = []
for i in range(6):
    rest._clock.t = i / 60.0
    firsts.append(round(rest.snapshot()["open"][-1], 3))
check("a sentence that starts loudly cannot pop the mouth open",
      firsts[0] < 0.35 and firsts == sorted(firsts), f"first frames {firsts}")

window = Speech(clock=Clock(0.0))
for i in range(WINDOW_N + 30):
    window._clock.t = i / 60.0
    window.snapshot()
check("the published curve is a rolling window, whatever the idle time",
      len(window.snapshot()["open"]) == WINDOW_N,
      f"got {len(window.snapshot()['open'])}")

# Idle for a long time, then speak: the curve must not be a huge catch-up burst
# and the jaw must still open on the first syllable.
after_idle = Speech(clock=Clock(0.0))
after_idle._clock.t = 600.0
st = after_idle.snapshot()
check("a long silence is re-anchored, not replayed",
      len(st["open"]) <= WINDOW_N, f"got {len(st['open'])}")
after_idle.begin("z", wav=make_wav(tone(0.2) + SILENT))
opened = 0.0
for i in range(20):
    after_idle._clock.t += 1.0 / 60.0
    opened = max(opened, after_idle.snapshot()["open"][-1])
check("the mouth still opens after a long silence", opened > 0.5, f"got {opened:.3f}")

# The app's poll thread and the TTS thread are different threads: a snapshot
# taken while an utterance starts must still be coherent.
shared = Speech(clock=Clock())
errors = []


def chatter():
    try:
        for _ in range(200):
            shared.begin("talking", wav=make_wav(tone(0.05)))
            shared.snapshot()
            shared.end()
    except Exception as e:                                       # noqa: BLE001
        errors.append(repr(e))


threads = [threading.Thread(target=chatter) for _ in range(4)]
for t in threads:
    t.start()
for t in threads:
    t.join()
check("starting, ending and polling from different threads is safe",
      not errors, f"got {errors[:1]}")


print()
print(f"{CHECKS - len(FAILURES)}/{CHECKS} checks passed")
if FAILURES:
    print(f"{len(FAILURES)} FAILED")
    raise SystemExit(1)
print("ALL SPEECH-FACE PROBES PASSED")
