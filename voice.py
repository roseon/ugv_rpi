"""The robot's voice: the one place that decides how Lance sounds.

Every path that makes the robot speak goes through here — the web UI's toggles,
``POST /api/say``, Lance's replies, the boot greeting, the person announcement
and the wake-word interaction.  Before this module three different voices came
out of the same robot: the plain Azure default in two call sites, the Minion
SSML in another, and pyttsx3 in two more, so "the robot's voice" depended on
which caller happened to be written first.

**Why the parameters look like this.** No commercial TTS ships the licensed
Minion voice — it is performed, in an invented language, and it is not a voice
token any service sells.  What makes speech read as a Minion is a small register
pushed far up in pitch, spoken quickly, with the odd Minion word dropped in.
That is the voice below.  It used to be four presets with a ``POST /voice``
picker; nobody asked for a picker and nothing in the UI could reach it, so the
voice is a definition now rather than a choice.

**One voice at a time.** A module-level lock owns that: two callers that used to
talk over each other (a UI toggle through audio_ctrl and Lance's reply through
cv_ctrl) now cannot, because the deciding is here rather than in each caller.

The audio the mouth is drawn from is measured from the very WAV played, so the
Pi 5 screen and the desktop both animate the syllables actually coming out.
"""

import logging
import os
import random
import subprocess
import tempfile
import threading
import urllib.request

import speech_face

# ── the service ───────────────────────────────────────────────────────────────
# Moved here from cv_ctrl so there is one owner of the credential as well as of
# the voice.  Worth rotating: it has been in the repository for months.
AZURE_KEY = os.environ.get("AZURE_SPEECH_KEY", "702d957143704526a6687ac6cde18194")
AZURE_REGION = os.environ.get("AZURE_SPEECH_REGION", "eastus2")

# ── the voice itself ──────────────────────────────────────────────────────────
# A voice token to carry the pitch and rate, the two prosody controls that make
# the register, and the interjections that make it sound like a Minion.
VOICE = "en-US-JennyNeural"
PITCH = "+40%"
RATE = "+28%"
MINION_PHRASES = ("Bello!", "Papoy!", "Bee-do-bee-do-bee-do!", "Ta-ta!",
                  "Banana!", "Underwear!")

_voice_lock = threading.Lock()          # one utterance at a time, robot-wide


def ssml(text):
    """The Minion SSML for `text` — the one definition of how a sentence sounds.

    A leading interjection is part of the voice, not part of the caller's text,
    so it is added here and the caller keeps speaking the words it meant.
    """
    spoken = "%s %s" % (random.choice(MINION_PHRASES), text)
    return (spoken,
            '<speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xml:lang="en-US">'
            '<voice name="%s"><prosody pitch="%s" rate="%s">%s</prosody></voice></speak>'
            % (VOICE, PITCH, RATE, spoken))


# ── synthesis and playback ────────────────────────────────────────────────────
def synth_rest(ssml_text, timeout=25):
    """WAV bytes from the Azure TTS REST endpoint.

    REST rather than the Speech SDK's WebSocket layer: measured on this robot,
    the WebSocket fails to open after a reboot while plain HTTPS works, so the
    SDK is not a fallback worth carrying.
    """
    url = "https://%s.tts.speech.microsoft.com/cognitiveservices/v1" % AZURE_REGION
    req = urllib.request.Request(url, data=ssml_text.encode("utf-8"), method="POST")
    req.add_header("Ocp-Apim-Subscription-Key", AZURE_KEY)
    req.add_header("Content-Type", "application/ssml+xml")
    req.add_header("X-Microsoft-OutputFormat", "riff-24khz-16bit-mono-pcm")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def play_wav(wav_bytes, timeout=40):
    """Play WAV bytes through the system default sink (the Bluetooth speaker)."""
    fd, path = tempfile.mkstemp(suffix=".wav", dir="/tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(wav_bytes)
        subprocess.run(["/usr/bin/paplay", path], timeout=timeout, check=False)
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def _speak_local(spoken):
    """Last resort with no network: a local engine, pitched the same way.

    espeak-ng if it is installed (high pitch and speed are exactly what its
    ``-p``/``-s`` control), otherwise pyttsx3.  Either way the mouth still moves,
    on the text-derived envelope, because there is no WAV to measure.
    """
    if subprocess.call(["/bin/sh", "-c", "command -v espeak-ng"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
        subprocess.run(["espeak-ng", "-v", "en+f3", "-p", "99", "-s", "200", spoken],
                       timeout=60, check=False)
        return True
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.setProperty("rate", 200)
        engine.say(spoken)
        engine.runAndWait()
        return True
    except Exception as e:                                  # noqa: BLE001
        logging.error("no way to speak on this robot: %s", e)
        return False


def speak(text, timeout=25):
    """Say `text` in the Minion voice.  Blocking; returns the words spoken.

    Returns None when another utterance already holds the voice — dropping a
    sentence is better than two voices talking over each other.
    """
    text = (text or "").strip()
    if not text:
        return None
    if not _voice_lock.acquire(blocking=False):
        print("Audio already playing; skipping speech.")
        return None
    try:
        spoken, ssml_text = ssml(text)
        try:
            wav = synth_rest(ssml_text, timeout)
            if wav and wav[:4] == b"RIFF":
                seq = speech_face.FACE.begin(spoken, wav)
                try:
                    play_wav(wav)
                finally:
                    speech_face.FACE.end(seq)
                return spoken
            logging.warning("REST TTS returned no audio; speaking locally")
        except Exception as e:                              # noqa: BLE001
            logging.warning("REST TTS error: %s; speaking locally", e)

        seq = speech_face.FACE.begin(spoken, None)
        try:
            _speak_local(spoken)
        finally:
            speech_face.FACE.end(seq)
        return spoken
    finally:
        _voice_lock.release()
