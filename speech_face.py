"""What the mouths draw: is the robot speaking, and exactly how its mouth moves.

One owner of that question, and one owner of **the motion** - the jaw, the
corner width and the smile.  Both faces read the curve computed here: the panel
on the robot's own screen and the desktop in the Command Center.  Until this
module owned it, the model was written twice, character for character, in
`MouthView.cs` and `face_screen.py`, which is a description of how two faces
drift apart.

The envelope is measured from the very WAV bytes that are about to be played, so
both mouths are driven by the robot's real audio: the loudness of each syllable,
at the moment it happens.  A timer pretending to talk looks like a wobbling
oval; this looks like speech.

The timeline belongs to the audio, not to the player.  paplay returns early when
no sink is connected, and a silent failure used to be indistinguishable from a
finished sentence, so the state keeps running until the envelope has been
consumed - GRACE_S past the end - whether or not a speaker was there to hear it.
That also means both mouths animate on a robot with no speaker at all.
"""

import collections
import math
import struct
import threading
import time

# Envelope resolution.  60 frames/s is display rate, which is what lets a face
# draw every syllable instead of interpolating between samples of a level.
ENV_FPS = 60.0

# ── the one definition of how the mouth moves ─────────────────────────────────
# Both faces read the curve these produce; the numbers exist only here.
SPS = 60                          # the curve's rate: display rate, same as audio
WINDOW_S = 1.5                    # how much of it travels in one status payload
WINDOW_N = int(SPS * WINDOW_S)

ATTACK_TAU = 0.045                # the jaw opens quickly...
RELEASE_TAU = 0.11                # ...and closes slower: most of what reads as speech
FAST_TAU, SLOW_TAU = 0.06, 0.35   # fast minus slow is a syllable's attack
OPEN_EXP = 0.75
WIDE_GAIN, WIDE_CLAMP = 1.6, 0.35
BREATH_BASE, BREATH_AMP, BREATH_HZ = 0.035, 0.02, 0.22
BREATH_PERIOD, BREATH_KICK = 6.5, 0.10
SMILE_IDLE, SMILE_SPEAKING = 0.16, 0.04

# pyttsx3's local voice has no WAV to measure, so its envelope is derived from
# the text: roughly this many characters per second, and a syllable per vowel
# run.  It exists so the mouth still moves when synthesis happens on the robot.
TEXT_CHARS_PER_S = 13.0

# A mouth that snaps shut the instant the audio ends reads as a glitch; a short
# tail lets it close.
GRACE_S = 0.35


def _wav_format(wav):
    """(channels, rate, bits, data offset, data bytes) for a PCM RIFF, else None.

    Only the formats the robot's TTS produces are read (`riff-24khz-16bit-mono-
    pcm`), plus 8-bit and stereo for tolerance.  Anything else returns None and
    the caller falls back to the text envelope rather than guessing at samples:
    reading a compressed or float stream as PCM would drive the mouth with noise.
    """
    if not wav or len(wav) < 44 or wav[:4] != b"RIFF" or wav[8:12] != b"WAVE":
        return None
    pos, end = 12, len(wav)
    fmt = None
    data_off = data_len = None
    while pos + 8 <= end:
        cid = wav[pos:pos + 4]
        size = struct.unpack_from("<I", wav, pos + 4)[0]
        body = pos + 8
        if cid == b"fmt " and size >= 16 and body + 16 <= end:
            audio_format, channels, rate = struct.unpack_from("<HHI", wav, body)
            bits = struct.unpack_from("<H", wav, body + 14)[0]
            if audio_format != 1:                      # not PCM: refuse it
                return None
            fmt = (channels, rate, bits)
        elif cid == b"data":
            data_off, data_len = body, min(size, max(0, end - body))
            break
        pos = body + size + (size & 1)               # chunks are word-aligned
    if fmt is None or data_off is None:
        return None
    channels, rate, bits = fmt
    if channels < 1 or rate <= 0 or bits not in (8, 16):
        return None
    return channels, rate, bits, data_off, data_len


def envelope_from_wav(wav, fps=ENV_FPS):
    """(levels, duration_s) measured from WAV bytes, or (None, None).

    Levels are normalised so the loudest syllable opens the mouth fully, then
    lifted by a square root: speech RMS is dominated by the vowel peaks, and
    without it the consonant and gap frames all sit at nearly zero and the lips
    look glued shut between syllables.
    """
    parsed = _wav_format(wav)
    if parsed is None:
        return None, None
    channels, sample_rate, bits, data_off, data_len = parsed
    width = bits // 8
    step = channels * width
    nsamples = data_len // step
    if nsamples <= 0:
        return None, None
    per_frame = max(1, int(round(sample_rate / fps)))
    levels = []
    total = 0.0
    count = 0
    for i in range(nsamples):
        off = data_off + i * step
        acc = 0
        for c in range(channels):
            p = off + c * width
            if bits == 8:
                acc += wav[p] - 128
            else:
                acc += struct.unpack_from("<h", wav, p)[0]
        v = acc / channels
        total += v * v
        count += 1
        if count == per_frame:
            levels.append(math.sqrt(total / count))
            total = 0.0
            count = 0
    if count:
        levels.append(math.sqrt(total / count))
    if not levels:
        return None, None
    peak = max(levels)
    if peak <= 0:
        return [0.0] * len(levels), nsamples / float(sample_rate)
    shaped = [math.sqrt(min(1.0, v / peak)) for v in levels]
    # 3-frame box blur: one frame of a syllable should not be a spike.
    smoothed = []
    for i, v in enumerate(shaped):
        lo = max(0, i - 1)
        hi = min(len(shaped), i + 2)
        smoothed.append(sum(shaped[lo:hi]) / (hi - lo))
    return smoothed, nsamples / float(sample_rate)


_VOWELS = set("aeiouy")


def envelope_from_text(text, fps=ENV_FPS):
    """(levels, duration_s) for speech with no WAV: a plausible syllable shape.

    Used when the robot synthesises locally.  Vowels open the mouth, everything
    else nearly closes it, and the whole thing rises and falls so the result
    reads as a sentence rather than as a mechanism.
    """
    text = (text or "").strip()
    duration = max(0.6, min(20.0, len(text) / TEXT_CHARS_PER_S))
    n = max(2, int(round(duration * fps)))
    levels = []
    for i in range(n):
        t = i / fps
        phase = t * 2.0 * math.pi * 4.2                       # ~4 syllables/s
        syllable = 0.5 + 0.5 * math.sin(phase)
        char = text[int(t * TEXT_CHARS_PER_S) % len(text)] if text else "a"
        vowel = 0.55 + 0.45 * syllable if char.lower() in _VOWELS else 0.30 * syllable
        arc = math.sin(math.pi * min(1.0, t / duration))       # ease in and out
        levels.append(min(1.0, vowel * arc * 1.15))
    return levels, duration


class Speech:
    """The robot's mouth over time: the curve both faces read, and its state.

    The filter runs at SPS on this clock rather than on each renderer's frame
    timing, so the same syllable produces the same motion on the panel and on the
    desktop.  It is advanced lazily - whoever polls next tops the curve up to
    now - which is why there is no drawing thread in this process.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._seq = 0
        self._text = ""
        self._source = ""
        self._env = []
        self._t0 = 0.0
        self._duration = 0.0
        self._playback_done = True
        # the model's state, and the window of what it has produced
        self._open = self._wide = self._smile = None
        self._fast = self._slow = 0.0
        self._next_t = None
        self._curve = collections.deque(maxlen=WINDOW_N)

    def begin(self, text, wav=None):
        """Start an utterance.  Returns its sequence number, for `end`."""
        levels, duration = (envelope_from_wav(wav) if wav else (None, None))
        source = "audio"
        if levels is None:
            levels, duration = envelope_from_text(text)
            source = "text"
        with self._lock:
            self._seq += 1
            self._text = (text or "").strip()
            self._source = source
            self._env = levels
            self._duration = duration
            self._t0 = self._clock()
            self._playback_done = False
            return self._seq

    def end(self, seq=None):
        """Playback returned.  The timeline still runs to the end of the audio.

        `seq` guards the common race: the player can return while a newer
        utterance is already speaking, and the older one must not close it.
        """
        with self._lock:
            if seq is not None and seq != self._seq:
                return
            self._playback_done = True

    # ── the model ────────────────────────────────────────────────────────────
    def _level(self, t):
        """What the audio is doing at time `t`, or the idle breath when it is not."""
        if self._template_speaking(t):
            idx = int((t - self._t0) * ENV_FPS)
            if 0 <= idx < len(self._env):
                return float(self._env[idx])
            return float(self._env[-1]) if self._env and t - self._t0 < self._duration else 0.0
        # idle: a slow breath, with a small yawn at the top of every cycle
        phase = (t % BREATH_PERIOD) / BREATH_PERIOD
        kick = BREATH_KICK * math.sin(math.pi * phase / 0.05) if phase < 0.05 else 0.0
        return BREATH_BASE + BREATH_AMP * math.sin(2 * math.pi * BREATH_HZ * t) + kick

    def _template_speaking(self, t):
        return bool(self._env) and (t - self._t0) < self._duration + GRACE_S

    def _step(self, t, dt):
        """One frame of the mouth: the five numbers that define how it moves."""
        level = self._level(t)
        speaking = self._template_speaking(t)
        if self._open is None:
            # First frame of this process: start from a resting mouth and let the
            # attack happen, as both faces did before they shared this model.  A
            # sentence that begins loudly must not pop open.
            self._open, self._wide, self._smile = BREATH_BASE, 0.0, SMILE_IDLE
            self._fast = self._slow = level
            self._curve.append((self._open, self._wide, self._smile))
            return
        self._fast += (level - self._fast) * (1 - math.exp(-dt / FAST_TAU))
        self._slow += (level - self._slow) * (1 - math.exp(-dt / SLOW_TAU))
        if speaking:
            target_open = level ** OPEN_EXP
            target_wide = max(-WIDE_CLAMP, min(WIDE_CLAMP, WIDE_GAIN * (self._fast - self._slow)))
            target_smile = SMILE_SPEAKING
        else:
            target_open = level
            target_wide = 0.0
            target_smile = SMILE_IDLE + 0.05 * math.sin(2 * math.pi * 0.09 * t)
        # jaws open faster than they close: most of what reads as speech
        tau = ATTACK_TAU if target_open > self._open else RELEASE_TAU
        self._open += (target_open - self._open) * (1 - math.exp(-dt / tau))
        self._wide += (target_wide - self._wide) * (1 - math.exp(-dt / 0.09))
        self._smile += (target_smile - self._smile) * (1 - math.exp(-dt / 0.25))
        self._curve.append((self._open, self._wide, self._smile))

    def _advance(self, now):
        """Top the curve up to `now`, one SPS-th of a second at a time."""
        if self._next_t is None:
            self._next_t = now
        steps = 0
        while self._next_t <= now and steps <= WINDOW_N:
            self._step(self._next_t, 1.0 / SPS)
            self._next_t += 1.0 / SPS
            steps += 1
        if steps > WINDOW_N:                       # idle for ages: re-anchor
            self._next_t = now + 1.0 / SPS

    def snapshot(self):
        """The status both faces poll: what to draw, and the curve to draw from."""
        with self._lock:
            now = self._clock()
            self._advance(now)
            text, source, seq = self._text, self._source, self._seq
            speaking = self._template_speaking(now)
            curve = list(self._curve)
        return {
            "speaking": speaking,
            "seq": seq,
            "text": text if speaking else "",
            "source": source if speaking else "",
            "sps": SPS,
            # How far back the curve above reaches.  The renderers need it to
            # decide when a status has gone stale, and it belongs to the model
            # rather than being re-derived as a constant in each of them.
            "window_s": WINDOW_S,
            "open": [round(f[0], 4) for f in curve],
            "wide": [round(f[1], 4) for f in curve],
            "smile": [round(f[2], 4) for f in curve],
        }


FACE = Speech()
