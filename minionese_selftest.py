#!/usr/bin/env python3
"""What the Minionese model has to do, checked against the corpus it was trained on.

Run: python minionese_selftest.py
"""

import re

import minionese
import voice

FAILURES = []
CHECKS = 0


def check(name, ok, detail=""):
    global CHECKS
    CHECKS += 1
    print(("  ok   " if ok else "  FAIL ") + name + ((" - " + str(detail)) if detail and not ok else ""))
    if not ok:
        FAILURES.append(name)


def _bare(text):
    """A phrase without its punctuation, lowercased, for comparing meanings."""
    return text.strip(".!,?").lower()


def main():
    lang = minionese.LANGUAGE
    print("== the corpus is trained, and every part of it is used ==")
    check("the corpus file loads", lang is not None)
    if lang is None:
        return 1
    check("vocabulary pairs parsed (>= 100)", lang.pairs >= 100, lang.pairs)
    check("lookup keys (>= 120)", len(lang.table) >= 120, len(lang.table))
    check("Minionese words learned (>= 80)", len(lang.vocab) >= 80, len(lang.vocab))
    check("opening words learned (>= 60)", len(lang.starts) >= 60, len(lang.starts))
    # both halves of the corpus must reach the table: the dash list and the
    # run-on block, which are parsed by different rules
    check("the dash list parsed (hello)", lang.table.get("hello") == "Bello", lang.table.get("hello"))
    check("the run-on block parsed (bonjour)", lang.table.get("bonjour") == "Bello",
          lang.table.get("bonjour"))

    print("\n== the words the corpus teaches ==")
    expected = {"hello": "Bello", "hey": "Ayoo", "goodbye": "Poopaye", "thank you": "Tank yu",
                "stop": "Stupa", "what": "Poka?", "apple": "Bapple", "ice cream": "Gelato",
                "i'm sorry": "Bi-do", "we love you": "Tulaliloo ti amo", "for you": "Para tu",
                "look at you": "Luk at tu", "i'm hungry": "Me want banana", "i swear": "Underwear",
                "fire": "Bee do bee do bee do", "cheers": "Kanpai", "merci": "Grazi",
                "please": None}
    for word, want in expected.items():
        got = lang.table.get(word)
        if want is None:
            continue
        check("%-14s -> %s" % (word, want), got == want, got)

    print("\n== numbers are spoken as Minionese ==")
    check("one/two/three from the English list",
          (lang.table.get("one"), lang.table.get("two"), lang.table.get("three")) == ("Hana", "dul", "sae"))
    check("four..ten from the French list",
          (lang.table.get("quatre"), lang.table.get("dix")) == ("Chari", "Ju"),
          (lang.table.get("quatre"), lang.table.get("dix")))
    digits = {d: lang.translate(d) for d in map(str, range(1, 11))}
    check("all ten digits translate to learned words",
          all(v and v.lower() in lang.vocab for v in digits.values()), digits)
    check("a digit in a sentence is spoken",
          lang.translate("3 bananas").split()[0].lower() in lang.vocab, lang.translate("3 bananas"))

    print("\n== every entry the corpus teaches is reachable ==")
    # Derived from the parsed corpus rather than from the table, so this proves the
    # entries themselves are reachable: every entry's taught side - English, French,
    # a phrase or a single word - became a key, and asking for it gives back that
    # entry's Minionese.
    first = {}          # taught key -> (entry index, that entry's Minionese)
    repeated = {}       # taught key the corpus uses for more than one Minionese
    for i, (minion, other) in enumerate(lang.entries):
        for key in (minionese._clean_key(o) for o in other.split("/")):
            if not key:
                continue
            if key in first and _bare(first[key][1]) != _bare(minion):
                repeated.setdefault(key, []).append(first[key][1])
            if key not in first:
                first[key] = (i, minion)
    missing = sorted(k for k in first if k not in lang.table)
    check("every one of the %d corpus entries taught a key" % len(lang.entries),
          not missing, missing[:4])
    wrong = {k: (lang.translate(k), mn) for k, (_i, mn) in first.items()
             if _bare(lang.translate(k)) != _bare(mn)}
    check("every taught key gives its entry's Minionese", not wrong, list(wrong.items())[:4])
    # Where the corpus repeats a taught word ("bonjour" is Bello, Aloha and
    # Konnichiwa), the first entry wins everywhere, so the word means one thing.
    drifted_keys = {k: v for k, v in repeated.items()
                    if _bare(lang.translate(k)) != _bare(first[k][1])}
    check("a taught word the corpus repeats still means one thing (%d repeated)"
          % len(repeated), not drifted_keys, drifted_keys)

    print("\n== the English column reaches the meanings the corpus teaches in French ==")
    # minionese_english.txt is the English side of the corpus's own meanings.  It
    # has to cover exactly those: one line too many is learned as a word (a colon
    # in the header did exactly that, costing "french (moka"), and one too few
    # leaves a meaning whose English words come out invented.
    check("the English column loaded (%d lines)" % len(lang.english), len(lang.english) >= 90,
          len(lang.english))
    corpus_sides = {_bare(mn) for mn, _o in lang.entries}
    column_sides = {_bare(mn) for mn, _o in lang.english}
    check("it covers every one of the %d corpus meanings" % len(corpus_sides),
          corpus_sides == column_sides, sorted(corpus_sides ^ column_sides)[:4])
    english_keys = {k for _mn, other in lang.english
                    for k in (minionese._clean_key(o) for o in other.split("/")) if k}
    unkeyed = sorted(k for k in english_keys if k not in lang.table)
    check("every one of its %d English words is a key" % len(english_keys), not unkeyed, unkeyed[:4])
    # It must not become a second author of the language: what it reaches has to be
    # the corpus's own Minionese.
    foreign = {k: lang.table[k] for k in english_keys
               if k in lang.table and _bare(lang.table[k]) not in corpus_sides}
    check("it only reaches Minionese the corpus taught", not foreign, foreign)
    for word, want in {"please": "Moka", "sorry": "Bi-do", "you're welcome": "Prego",
                       "let's go": "Vamo", "follow me": "Chupa", "low battery": "Pip pip pip",
                       "listen": "Tara", "four": "Chari", "ten": "Ju", "yes": "Si",
                       "window": "Moka", "danger": "Whaaa", "chair": "Soka"}.items():
        got = lang.translate(word)
        check("%-15s -> %-14s" % (word, want), _bare(got) == _bare(want), got)

    # Words that only one entry uses, which is how the corpus's own Minionese
    # reaches them instead of an invented word.
    taught = {"sorry": "Bi-do", "hungry": "Me want banana", "thank": "Tank yu",
              "love": "Tulaliloo ti amo", "look": "Luk at tu", "swear": "Underwear",
              "dare": "Sa la ka!", "ice": "Gelato", "cream": "Gelato",
              "plaît": "Moka", "désolé": "Bi-do", "quatre": "Chari", "aide": None}
    for word, want in taught.items():
        if want is None:
            continue
        got = lang.translate(word)
        check("%-8s -> %-18s" % (word, want), got.strip(".!,?").lower() == want.strip(".!,?").lower(), got)

    # The other side of every entry: Minionese in, the same Minionese out, because
    # it is the vocabulary and translate() keeps those words.  An entry whose own
    # Minionese text is another entry's taught side ("Kiss kiss – Muak muak muak")
    # is translated on purpose, so it is not a failure.
    taught_keys = set(lang.table)
    passthrough = {v: lang.translate(v) for v in set(lang.table.values())
                   if _bare(v) not in taught_keys
                   and _bare(lang.translate(v)) != _bare(v)}
    check("every Minionese side passes through unchanged", not passthrough, passthrough)

    # One word, one Minion word: the same taught word cannot mean two things
    # depending on what is around it.
    drifted = {w: (lang.translate(w), lang.translate("Bob says %s now" % w))
               for w in ("sorry", "hungry", "thank you", "stop", "fire")
               if lang.translate(w).lower() not in lang.translate("Bob says %s now" % w).lower()}
    check("a taught word means the same alone and in a sentence", not drifted, drifted)

    # A word several entries would answer with is left to the invented path rather
    # than resolved by whichever entry came first.
    check("'you' is not resolved by chance", "you" not in lang.table, lang.table.get("you"))
    check("and it still becomes one Minion word",
          len(lang.translate("you").split()) == 1, lang.translate("you"))

    print("\n== the grammar the corpus shows ==")
    dropped = lang.translate("the robot is a machine")
    check("articles and copulas are dropped", not re.search(r"\b(the|is|a)\b", dropped, re.I), dropped)
    # A taught word comes out as Minionese.  Not "it must look different": banana
    # is Minionese already, and so are kanpai and six - what matters is that every
    # word of the result is one the corpus uses.
    wrong = {}
    for word in ("banana", "hello", "stop", "fire", "apple", "thank you", "goodbye",
                 "cheers", "i'm sorry", "we love you", "ice cream"):
        out = lang.translate(word)
        # The model's own idea of a word: "Bi-do" is one, not two.
        if not out or not all(t.lower() in lang.vocab for t in minionese._WORD.findall(out)):
            wrong[word] = out
    check("a taught word comes out as Minionese", not wrong, wrong)

    print("\n== the language is productive (a word the corpus never had) ==")
    battery = lang.translate("battery")
    check("an unseen word becomes a Minion word", battery.lower() != "battery", battery)
    check("it is Minion-shaped (4..8 letters, corpus alphabet)",
          4 <= len(battery) <= 8 and set(battery.lower()) <= lang.char.alphabet,
          "%r %s" % (battery, sorted(set(battery.lower()) - lang.char.alphabet)))
    check("the same word always comes out the same way",
          lang.translate("battery") == battery and lang.translate("Battery") == battery,
          lang.translate("battery"))
    check("two different words do not collapse into one",
          lang.translate("battery") != lang.translate("wheel"))

    print("\n== what the robot would actually say ==")
    line = lang.speak("Hello! Thank you for the banana.")
    check("the opening word is one the corpus uses",
          line.split()[0].lower() in lang.starts, line)
    check("the line is Minionese, not English",
          "thank" not in line.lower().split() and "banana" in line.lower(), line)
    openings = {lang.speak("hello").split()[0] for _ in range(60)}
    check("it does not open every sentence the same way", len(openings) >= 5, sorted(openings))
    for sentence in ("The battery is low, please charge me.",
                     "I see a person 3 metres ahead.",
                     "Stop, there is a wall in front of you."):
        spoken = lang.speak(sentence)
        print("    %-42s -> %s" % (sentence, spoken))
        words = len(sentence.split())
        check("  the line stays a sentence (%s)" % sentence[:18], 2 <= len(spoken.split()) <= words + 6,
              len(spoken.split()))

    print("\n== edges ==")
    for bad in ("", "   ", None):
        check("empty input says nothing (%r)" % (bad,), minionese.speak(bad) == "")
    check("punctuation alone is left alone", lang.translate("!!!") == "!!!", repr(lang.translate("!!!")))
    check("a sentence keeps its own punctuation",
          lang.translate("Stop, now!").endswith("!"), repr(lang.translate("Stop, now!")))
    check("a missing corpus degrades to plain text instead of crashing",
          minionese.Minionese("").translate("hello there") != "")

    print("\n== the voice speaks it ==")
    spoken, ssml_text = voice.ssml("The battery is low, please charge me.")
    check("the voice's words are Minionese",
          not re.search(r"\b(the|is|please)\b", spoken, re.I), spoken)
    check("the opening word is the corpus's", spoken.split()[0].lower() in lang.starts, spoken)
    check("the SSML is still the Minion register",
          'pitch="+40%"' in ssml_text and 'rate="%s"' % voice.RATE in ssml_text)
    # The pitch carries the register; the rate is what the user hears as speed.
    # Pinned at or below natural so a future edit cannot quietly rush it again.
    check("and it is not sped up past natural", float(voice.RATE.strip("%")) <= 0, voice.RATE)
    check("the SSML carries the words it will say", spoken in ssml_text, spoken)
    check("an empty request stays empty", voice.ssml("")[0] == "")

    print("\n%d/%d checks passed" % (CHECKS - len(FAILURES), CHECKS))
    if FAILURES:
        print("FAILED: %s" % ", ".join(FAILURES))
        return 1
    print("ALL MINIONESE PROBES PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
