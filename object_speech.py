"""What the robot says about the things it recognises, in Minionese.

Detections arrive from every pass that looks with the camera -- the eyes' gaze
(every ``cam_hz``) and the object-detection CV mode -- as
``[{"name", "conf"|"confidence", "box"}]``.  This module owns three things and
nothing else does:

* **the reaction** -- what a Minion says about an object of that kind.  Words
  come from ``minionese.py``; the corpus is the only source of them, which is
  what keeps the robot recognisably Minion instead of the character model
  spraying syllables.  A reaction is a whole taught phrase, so it is right even
  where the noun is not.
* **the name** -- the object's own word.  The corpus teaches a word for nine of
  the robot's 468 detectable names (``chair`` is ``Soka``, ``banana`` is
  ``Banana``); where it teaches nothing, ``minionese`` invents one and
  ``source()`` says so, so an invented line is never passed off as taught.
* **when to say it** -- one line at a time, never the same object twice in a
  row, each object at most once per ``LABEL_COOLDOWN_S``, and never over the
  robot's voice.  A chair that stays in view says its word once, not forever.

The vocabulary is not repeated here.  ``labels()`` reads it from its owner
(``cv_ctrl``: the detector's names plus everything the robot has learned), so a
newly taught object is speakable without a second list to keep in step.
"""

import time

import minionese

# Detections weaker than this are noise the robot should not narrate.
CONF_FLOOR = 0.40

# The same object may be named again after this long -- long enough that a chair
# in view says its word once rather than every detection pass.
LABEL_COOLDOWN_S = 90.0

# ...and one line per this long, whatever is in view, so a scene full of objects
# is a robot that comments rather than one that talks without pause.
GAP_S = 6.0

# A detector that cannot name a class emits this placeholder, not a name.
_NOT_A_NAME = "class_"


# What a Minion says first, by kind.  The tuples are explicit names rather than
# substrings on purpose: "mouse" is a computer mouse in COCO and "fire hydrant"
# is not a fire, and a substring rule would kiss both.
REACTIONS = (
    ("person", ("person", "people", "man", "woman", "boy", "girl", "human",
                "guy", "lady", "child", "baby", "hands", "face"), "hello"),
    ("animal", ("bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
                "bear", "zebra", "giraffe", "fish", "duck", "chicken", "pet"), "muak muak muak"),
    ("danger", ("knife", "scissors", "axe", "razor", "sword", "saw", "drill",
                "gun", "pistol", "weapon"), "danger"),
    ("vehicle", ("car", "bus", "truck", "train", "airplane", "motorcycle",
                 "bicycle", "boat", "van", "taxi", "tractor", "scooter",
                 "ambulance"), "look at that"),
)

# The class is called "person"; the corpus already teaches what a Minion calls a
# person, and it is a better name than anything this module would invent.  An
# alias only changes the word, never the reaction.
ALIASES = {name: "boss" for name in REACTIONS[0][1]}


def _key(label):
    """One spelling per object, whatever case or spacing a detector reports."""
    return " ".join(str(label or "").split()).lower()


def _word(label):
    """The corpus word (or its absence) for an object's own name."""
    return ALIASES.get(label, label)


def _reaction(label):
    """The corpus phrase for this kind of object, or '' for the rest."""
    for _kind, names, phrase in REACTIONS:
        if label in names:
            return phrase
    return ""


def english(label):
    """The sentence handed to the voice: the reaction, then what it is."""
    label = _key(label)
    phrase = _reaction(label)
    word = _word(label)
    return "%s %s" % (phrase, word) if phrase else word


def line(label):
    """The Minionese the robot says for an object -- the table's spoken line.

    One translation of the whole sentence, so it cannot drift from what the
    voice says: ``voice.speak(english(label))`` completes this with its own
    opening word, exactly as every other sentence the robot speaks.
    """
    return minionese.translate(english(label))


def source(label):
    """``taught`` when the corpus has a word for this object, else ``invented``."""
    label = _key(label)
    return "taught" if _word(label) in minionese.LANGUAGE.table else "invented"


def labels(cvf=None):
    """Every name this robot can detect, from the detector's own vocabulary.

    ``cv_ctrl`` owns it: the learned/known object list it embeds for the
    open-vocabulary model, plus whatever the closed-set model it loaded emits.
    Sorted so the table and its counts are stable between calls.
    """
    names = set()
    if cvf is not None:
        names.update(_key(n) for n in getattr(cvf, "known_objects", None) or ())
        model = getattr(cvf, "yolo_model", None)
        names.update(_key(n) for n in (getattr(model, "names", None) or {}).values())
    return tuple(sorted(n for n in names if n and not n.startswith(_NOT_A_NAME)))


def table(names):
    """The library as rows: one per name, with the line the robot speaks for it."""
    return [{"name": n, "english": english(n), "line": line(n),
             "source": source(n), "reaction": _reaction(_key(n))}
            for n in names]


class ObjectSpeech:
    """The one owner of when the robot names an object, and of what it last said.

    ``choose`` is called on every detection pass and answers with the sentence
    to speak, or ``None`` to stay quiet.  It remembers what it returned, so the
    caller only has to speak it -- and so the robot cannot repeat itself while
    an object stays in view.
    """

    def __init__(self, label_cooldown=LABEL_COOLDOWN_S, gap=GAP_S,
                 conf_floor=CONF_FLOOR, clock=time.time):
        self.label_cooldown = float(label_cooldown)
        self.gap = float(gap)
        self.conf_floor = float(conf_floor)
        self._clock = clock
        self._last_any = None             # time of the last line spoken
        self._spoken_at = {}              # label -> time it was last named
        self._last_label = None           # so the same object never repeats
        self._last_line = None

    def choose(self, detections, busy=False):
        """The English line for the most salient detection, or None to stay quiet.

        Quiet while the robot is already speaking (``busy``), inside the gap
        since the last line, or for an object named too recently.
        """
        if busy:
            return None
        now = self._clock()
        if self._last_any is not None and now - self._last_any < self.gap:
            return None
        label = self._salient(detections, now)
        if label is None:
            return None
        self._last_any = now
        self._spoken_at[label] = now
        self._last_label = label
        self._last_line = line(label)
        return english(label)

    def _salient(self, detections, now):
        """The label worth naming: a person first, then the largest thing.

        People first because a room's largest box is usually furniture, and the
        robot naming a chair while someone stands in front of it is the report
        this ordering answers.  With nobody in frame the most prominent object
        is still better than silence.
        """
        best = None                       # (is_person, area, label)
        for det in detections or ():
            label = _key(det.get("name"))
            if not label or label.startswith(_NOT_A_NAME):
                continue  # an unnamed class, not an object to talk about
            conf = det.get("conf", det.get("confidence"))
            if not isinstance(conf, (int, float)) or conf < self.conf_floor:
                continue
            if label == self._last_label:
                continue                  # never the same object twice in a row
            seen = self._spoken_at.get(label)
            if seen is not None and now - seen < self.label_cooldown:
                continue
            try:
                x1, y1, x2, y2 = (int(v) for v in list(det.get("box") or ())[:4])
            except (TypeError, ValueError):
                x1 = y1 = x2 = y2 = 0   # a hit without a usable box is still a hit
            area = max(0, x2 - x1) * max(0, y2 - y1)
            person = label in REACTIONS[0][1]
            if best is None or (person, area) > (best[0], best[1]):
                best = (person, area, label)
        return best[2] if best else None

    def last(self):
        """What was named last and how long ago, for the status surface."""
        if self._last_label is None:
            return None
        return {"name": self._last_label, "line": self._last_line,
                "ago": round(self._clock() - self._last_any, 1)}

    def report(self, cvf=None):
        """The whole library as the robot knows it, and what it said last."""
        rows = table(labels(cvf))
        taught = sum(1 for row in rows if row["source"] == "taught")
        return {"labels": len(rows), "taught": taught,
                "invented": len(rows) - taught,
                "reactions": sum(1 for row in rows if row["reaction"]),
                "last": self.last(), "rows": rows}
