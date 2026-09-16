#!/usr/bin/env python3
"""What the robot says about the objects it recognises, and when it may say it.

Run: python object_speech_selftest.py
"""

import json
import os

import minionese
import object_speech

FAILURES = []
CHECKS = 0

# A vocabulary with the shapes the real one has: single words, a name the corpus
# teaches, a multiword name, and a class placeholder a detector can emit.
VOCAB = ["person", "chair", "wall", "flower", "potted plant", "traffic light",
         "banana", "dog", "class_7", "ice cream"]


def check(name, ok, detail=""):
    global CHECKS
    CHECKS += 1
    print(("  ok   " if ok else "  FAIL ") + name + ((" - " + str(detail)) if detail and not ok else ""))
    if not ok:
        FAILURES.append(name)


class FakeCV:
    """Stands in for cv_ctrl: the vocabulary owner, without loading a model."""

    def __init__(self, objects=(), names=()):
        self.known_objects = list(objects)
        self.yolo_model = type("M", (), {"names": {i: n for i, n in enumerate(names)}})()


class Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def main():
    print("== every name gets a line, and the line is the language's ==")
    rows = {row["name"]: row for row in object_speech.table(VOCAB)}
    check("every name in the vocabulary appears in the table", len(rows) == len(VOCAB), len(rows))
    check("no line is empty", all(row["line"].strip() for row in rows.values()))
    check("a line is the voice's own translation, so the two cannot drift",
          all(row["line"] == minionese.translate(row["english"]) for row in rows.values()))
    check("every row says where its word came from",
          all(row["source"] in ("taught", "invented") for row in rows.values()))

    print("\n== the corpus is the word source, and reactions are whole taught phrases ==")
    check("a chair is the corpus's word", rows["chair"]["line"] == "Soka" and rows["chair"]["source"] == "taught",
          rows["chair"])
    check("a person is the corpus's word for a person, not an invention",
          rows["person"]["source"] == "taught"
          and rows["person"]["line"].lower().endswith("boss"), rows["person"])
    check("the corpus knows no word for a wall, and says so",
          rows["wall"]["source"] == "invented" and rows["wall"]["line"] != "", rows["wall"])
    check("an invented word is stable, not sampled per call",
          object_speech.line("wall") == object_speech.line("wall") == rows["wall"]["line"])
    check("every reaction is a phrase the corpus taught",
          all(phrase in minionese.LANGUAGE.table
              for _kind, _names, phrase in object_speech.REACTIONS))
    check("each name belongs to at most one kind, so a kind is never ambiguous",
          all(sum(1 for _k, names, _p in object_speech.REACTIONS if n in names) <= 1
              for _k, names, _p in object_speech.REACTIONS for n in names))
    check("a dog is greeted the way a Minion greets an animal",
          object_speech.english("dog").startswith("muak muak muak"), object_speech.english("dog"))
    check("a knife is treated as dangerous", object_speech.english("knife").startswith("danger"))
    check("a car is pointed at", object_speech.english("car").startswith("look at that"))
    check("an object of no particular kind just gets its name",
          object_speech.english("chair") == "chair")

    print("\n== the vocabulary is read from its owner, not repeated here ==")
    stub = FakeCV(objects=["Chair", "chair", "flower", "class_7"], names=(0, "Person", "Bicycle"))
    check("the detector's names and the learned list are merged, lowercased, deduped",
          object_speech.labels(stub) == ("bicycle", "chair", "flower", "person"),
          object_speech.labels(stub))
    check("a class placeholder is not a name", "class_7" not in object_speech.labels(stub))
    check("no vocabulary at all is not an error", object_speech.labels() == ())

    print("\n== the library can be read whole ==")
    speech = object_speech.ObjectSpeech(clock=Clock())
    report = speech.report(FakeCV(objects=["chair", "wall", "banana", "dog"]))
    check("the report counts every row it shows", report["labels"] == len(report["rows"]) == 4, report["labels"])
    check("taught and invented account for all of them",
          report["taught"] + report["invented"] == report["labels"],
          (report["taught"], report["invented"]))
    check("it says how many objects get a reaction", report["reactions"] == 1, report["reactions"])
    check("nothing has been said yet", report["last"] is None)

    print("\n== when the robot may speak ==")
    clock = Clock()
    speech = object_speech.ObjectSpeech(clock=clock)
    check("an empty view is silence", speech.choose([]) is None)
    check("a weak detection is not narrated",
          speech.choose([{"name": "chair", "conf": 0.2, "box": (0, 0, 10, 10)}]) is None)
    chair = [{"name": "chair", "conf": 0.9, "box": (0, 0, 50, 50)}]
    text = speech.choose(chair)
    check("a good detection is named", text == "chair", text)
    check("and remembered as spoken", speech.last()["line"] == "Soka", speech.last())
    check("an object already named is not named again on the next pass",
          speech.choose(chair) is None)
    check("nor a different object straight away",
          speech.choose([{"name": "banana", "conf": 0.9, "box": (0, 0, 40, 40)}]) is None)
    clock.advance(object_speech.GAP_S + 0.1)
    check("after the gap, something else in view is named",
          speech.choose([{"name": "banana", "conf": 0.9, "box": (0, 0, 40, 40)}]) == "banana")
    clock.advance(object_speech.GAP_S + 0.1)
    check("the same object stays quiet inside its cooldown", speech.choose(chair) is None)
    clock.advance(object_speech.LABEL_COOLDOWN_S)
    check("and may be named again after it", speech.choose(chair) == "chair")
    check("the robot does not speak over itself", speech.choose(chair, busy=True) is None
          and speech.choose(chair) is None)   # ...and that pass did not count as speech

    print("\n== what gets named when several things are in view ==")
    clock = Clock()
    speech = object_speech.ObjectSpeech(clock=clock)
    view = [{"name": "couch", "conf": 0.95, "box": (0, 0, 400, 400)},
            {"name": "person", "conf": 0.55, "box": (0, 0, 40, 40)}]
    chosen = speech.choose(view)
    check("a person outranks the bigger thing", chosen == "hello boss", chosen)
    clock = Clock()
    speech = object_speech.ObjectSpeech(clock=clock)
    check("with nobody in view the biggest thing is named",
          speech.choose([{"name": "couch", "conf": 0.95, "box": (0, 0, 400, 400)},
                         {"name": "bottle", "conf": 0.9, "box": (0, 0, 20, 20)}]) == "couch")
    clock = Clock()
    speech = object_speech.ObjectSpeech(clock=clock)
    check("the CV mode's 'confidence' spelling reads the same as 'conf'",
          speech.choose([{"name": "Couch", "confidence": 0.95, "box": (0, 0, 40, 40)}]) == "couch")
    clock = Clock()
    speech = object_speech.ObjectSpeech(clock=clock)
    check("a hit with no usable box is still a hit",
          speech.choose([{"name": "chair", "conf": 0.9}]) == "chair")
    check("a nameless class is not something to talk about",
          object_speech.ObjectSpeech(clock=Clock()).choose(
              [{"name": "class_3", "conf": 0.99, "box": (0, 0, 90, 90)}]) is None)

    print("\n== the table on a robot that has the real vocabulary ==")
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "known_objects.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fp:
            robot = FakeCV(objects=json.load(fp)["objects"])
        rows = object_speech.table(object_speech.labels(robot))
        taught = [r for r in rows if r["source"] == "taught"]
        print("  vocabulary %d names: %d taught by the corpus, %d invented, %d with a reaction"
              % (len(rows), len(taught), len(rows) - len(taught),
                 sum(1 for r in rows if r["reaction"])))
        check("the robot's own vocabulary is covered end to end",
              len(rows) >= 400 and all(r["line"].strip() for r in rows))
        check("the corpus-teaching is reported, not implied",
              any(r["name"] == "chair" for r in taught))
    else:
        print("  (skipped: known_objects.json is not in the tree -- it is runtime state)")

    print("\n%d/%d checks passed" % (CHECKS - len(FAILURES), CHECKS))
    if FAILURES:
        print("FAILED: %s" % ", ".join(FAILURES))
        return 1
    print("ALL OBJECT SPEECH PROBES PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
