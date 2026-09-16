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
pushed far up in pitch; the *language* is ``minionese.py``,
trained from the corpus the user supplied, and it is what turns a sentence into
the words below.  It used to be six hardcoded interjections glued in front of the
caller's English, which is English with a costume on, not Minionese.  A ``POST
/voice`` picker with four presets came and went too: nobody asked for a picker
to choose the voice, so the voice is a definition now rather than a choice.

**One voice at a time.** A module-level lock owns that: two callers that used to
talk over each other (a UI toggle through audio_ctrl and Lance's reply through
cv_ctrl) now cannot, because the deciding is here rather than in each caller.

The audio the mouth is drawn from is measured from the very WAV played, so the
Pi 5 screen and the desktop both animate the syllables actually coming out.
"""

import logging
import os
import subprocess
import tempfile
import threading
import urllib.request

import minionese
import speech_face

# ── the service ───────────────────────────────────────────────────────────────
# Moved here from cv_ctrl so there is one owner of the credential as well as of
# the voice.  Worth rotating: it has been in the repository for months.
AZURE_KEY = os.environ.get("AZURE_SPEECH_KEY", "702d957143704526a6687ac6cde18194")
AZURE_REGION = os.environ.get("AZURE_SPEECH_REGION", "eastus2")

# ── the voice itself ──────────────────────────────────────────────────────────
# The register is the pitch: pushed far up, a small voice.  The rate is *not*
# pushed up with it any more -- measured on this robot the same sentence took
# 7.6 s at +28% against 11.0 s at -10%, and at the fast end the corpus's own
# words ran together into something the user could not follow ("talking way too
# fast").  Slower costs nothing to keep in step: the mouth is drawn from the WAV
# that plays, so the face follows the voice wherever the rate goes.
VOICE = "en-US-JennyNeural"
PITCH = "+40%"
RATE = "-10%"

_voice_lock = threading.Lock()          # one utterance at a time, robot-wide


def ssml(text):
    """The Minion SSML for `text` — the one definition of how a sentence sounds.

    The words are not the caller's English: they are the Minionese that
    ``minionese.py`` makes of it, opening word included, so a sentence the robot
    is asked to say comes out in the corpus's own language.  The caller keeps the
    meaning it meant; if there is no corpus the text is spoken as it stands
    rather than dropped.
    """
    spoken = minionese.speak(text)
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


def play_wav(wav_bytes, timeout):
    """Play WAV bytes through the system default sink (the Bluetooth speaker).

    `timeout` is the caller's, and it has to follow the audio: this used to be a
    fixed 40 s, which covered about 78 words at the old rate and covers about 55
    at the current one, so a long answer would have been killed mid-sentence.
    """
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

    espeak-ng if it is installed (high pitch is what its ``-p`` controls, and
    ``-s`` is the words-per-minute to match the main voice's slower tempo -- 150
    reads at about the pace measured above), otherwise pyttsx3.  Either way the
    mouth still moves, on the text-derived envelope, because there is no WAV to
    measure.
    """
    if subprocess.call(["/bin/sh", "-c", "command -v espeak-ng"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
        subprocess.run(["espeak-ng", "-v", "en+f3", "-p", "99", "-s", "150", spoken],
                       timeout=60, check=False)
        return True
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.setProperty("rate", 150)
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
                    # 30 s of slack over the audio's own length, measured by the
                    # module that reads the WAV for the mouth.
                    _levels, audio_s = speech_face.envelope_from_wav(wav)
                    play_wav(wav, timeout=(audio_s or 0.0) + 30.0)
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
