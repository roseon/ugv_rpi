"""Speak Minionese: the one owner of what the robot says when it speaks like a Minion.

The words are the part no TTS service can supply, so they are trained here from
``minionese_corpus.txt`` — the transcript and vocabulary the user supplied — and
from ``minionese_english.txt``, the English side of the meanings that corpus
teaches only in French.  Three things come out of that training pass:  * **a table**, learned from the corpus's own vocabulary lines (``Bello – Hello``,
  ``Tank yu – Thank you``, ``Bee do bee do bee do – Fire``).  English and French
  both map to Minionese, because the corpus teaches both; a pair whose two sides
  are lists of the same length is split item by item, which is how ``Hana, dul,
  sae – One, two, three`` teaches the numerals one word at a time.  Both sides of
  every entry is reachable: the taught side and every word only one entry uses,
  which is how ``sorry`` (from ``I'm sorry``) reaches ``Bi-do`` instead of being
  invented, and the Minionese side passes through unchanged because it is the
  vocabulary the words are checked against.  The English column is what reaches
  the meanings the corpus teaches only in French; it is merged so the corpus wins
  every key it taught itself.
* **a start distribution** over the Minionese phrases, so the word the robot opens
  a sentence with is sampled from the corpus rather than from a list of six
  hardcoded interjections.
* **a character model** (order 3, backed off to order 2) built from the corpus's
  Minionese words, which is what makes the language productive: a word the table
  has never seen becomes a Minion word assembled from the corpus's own letters,
  seeded by that English word, so the same word always comes out the same way.

Grammar comes with the corpus too: Minionese drops articles and copulas (it says
``Me want banana`` and ``Me Tim!``, not ``I am Tim``), so ``the``/``is`` are
dropped rather than translated, and digits are spoken as the corpus's numerals.

``voice.py`` is the only caller.  It keeps owning how the robot sounds — the
pitch, the rate, one utterance at a time — and asks here for the words.

Run ``python minionese_selftest.py`` for the checks.
"""

import collections
import hashlib
import os
import random
import re

CORPUS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "minionese_corpus.txt")
ENGLISH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "minionese_english.txt")

# Minionese has no articles or copulas: "Me want banana", "Me Tim!".
_DROP = frozenset("the a an is are am was were be been being of".split())

# A word (hyphens included, so "Bi-do" survives), a digit, or one punctuation mark.
_WORD_PATTERN = r"[A-Za-z\u00c0-\u00ff']+(?:-[A-Za-z\u00c0-\u00ff']+)*"
_WORD = re.compile(_WORD_PATTERN)
_TOKEN = re.compile(r"%s|\d+|[^\sA-Za-z\u00c0-\u00ff\d]" % _WORD_PATTERN)

# The vocabulary section of the corpus is pasted as one run-on line, with the next
# Minionese word glued to the end of the previous translation ("Au revoirPoopaye"),
# so the pairs are cut apart before they are read.  Its section headings are glued
# the same way ("...DixCommandes de Mouvement et d'ActionLuk at tu : ...") and
# carry no colon of their own, so they are recognised by name - they are the
# corpus's own words, and without this the numerals 4-10 are lost to them.
_GLUE = re.compile(r"(?<=[^\s])(?=[A-Z][^:\n]{0,24}?:\s)")
_HEADINGS = ("Salutations et Civilités", "Chiffres et Mesures",
             "Commandes de Mouvement et d'Action",
             "Nourriture et Boissons (Idéal pour des modes de charge ou d'énergie)",
             "Objets et Environnement", "Émotions et États d'Esprit",
             "Alertes et Urgences",
             "Anatomie et Robotique (Idéal pour les capteurs)",
             "Expressions Courantes et Diverses")
# Only a real dash separates a pair.  A plain hyphen does not: the corpus writes
# pairs as "Bello – Hello" but also has Minionese words with hyphens ("Bi-do"),
# and a class containing "-" split those into nonsense pairs.
_EN_DASH = re.compile(r"^(?P<minion>.+?)\s*[\u2013\u2014]\s*(?P<other>.+)$")
_COLON = re.compile(r"^(?P<minion>[^:]+?)\s*:\s*(?P<other>.+)$")
_PARENTHETICAL = re.compile(r"\([^)]*\)")
_NON_WORD = re.compile(r"[^A-Za-z\u00c0-\u00ff' ]+")


def _split_pairs(line):
    """Cut a run-on vocabulary line into one pair per line.

    The corpus glues the next entry onto the end of the previous translation
    ("Au revoirPoopaye"), so the line has to be cut.  The cuts must land outside
    the parentheses: one that lands inside loses an entry's translation and
    corrupts the next entry as well — measured on ``Sous-vêtement (utilisé aussi
    pour dire ...)``, which cost that entry and mangled the one after it.
    """
    stashed = []

    def stash(match):
        stashed.append(match.group(0))
        return "\x00%d\x00" % (len(stashed) - 1)

    cut = _GLUE.sub("\n", re.sub(r"\([^()]*\)", stash, line))
    return re.sub(r"\x00(\d+)\x00", lambda m: stashed[int(m.group(1))], cut)


def _clean_key(text):
    """A lookup key: lowercase, no parentheticals, apostrophes straightened."""
    text = _PARENTHETICAL.sub(" ", text)
    text = text.replace("\u2019", "'").replace("\u2018", "'")
    text = _NON_WORD.sub(" ", text.lower())
    return " ".join(text.split())


# Words that carry no meaning on their own.  Indexing them by entry would drop a
# whole phrase into the middle of a sentence ("we" -> "Tulaliloo ti amo"), so they
# are skipped when the words of an entry are indexed.  Whole-entry keys are not
# touched by this: "for you" is still a phrase, and so is "i'm sorry".
_FUNCTION = frozenset("""
    i i'm you we me my your our he she it they them at to for of on in and or but
    not no is are am was were be the a an do does did that this with so as if then
    than there here
""".split())


def _pairs_from(minion, other):
    """One or more (minionese, translation) pairs from one vocabulary line."""
    left = [p.strip() for p in minion.split(",") if p.strip()]
    right = [p.strip() for p in other.split(",") if p.strip()]
    if len(left) > 1 and len(left) == len(right):
        return list(zip(left, right))
    return [(minion.strip(), other.strip())]


def _learn(text, index_words=True):
    """Train from one text: the table, the vocabulary, the start distribution.

    ``index_words`` is what separates the corpus from its English column.  The
    corpus teaches *words* (``I'm sorry`` is how ``sorry`` is taught), so a word
    only one of its entries uses becomes a key.  The English column teaches whole
    *meanings* (``Pip pip pip : low battery``), and letting it index words claims
    phrases for their fragments - measured, ``battery`` answered with
    ``Pip pip pip`` and left the character model with nothing to invent.  Its own
    meanings are still keys; only the fragment keys are withheld.
    """
    table = {}                       # cleaned key -> Minionese phrase
    vocab = collections.Counter()    # every Minionese word the corpus uses
    starts = collections.Counter()   # the word a Minionese phrase opens with
    entries = []                     # (Minionese side, taught side), in corpus order
    pairs = 0
    for heading in _HEADINGS:
        text = text.replace(heading, "\n" + heading + "\n")
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("-"):
            continue
        for chunk in _split_pairs(line).splitlines():
            chunk = chunk.strip()
            if not chunk:
                continue
            m = _EN_DASH.match(chunk) or _COLON.match(chunk)
            if not m:
                continue
            for minion_side, other_side in _pairs_from(m.group("minion"), m.group("other")):
                # A translation may offer several words ("Bonjour / Salut"): each
                # of them means the same Minionese phrase.
                keys = [k for k in (_clean_key(o) for o in other_side.split("/")) if k]
                if not keys or not _clean_key(minion_side):
                    continue
                pairs += 1
                entries.append((minion_side.strip(), other_side))
                for key in keys:
                    table.setdefault(key, minion_side)
                words = _WORD.findall(minion_side)
                if not words:
                    continue
                vocab.update(w.lower() for w in words)
                starts[words[0].lower()] += 1
    # And every word a taught entry uses, when exactly one entry uses it: that is
    # what makes "sorry" (from "I'm sorry") and "hungry" (from "I'm hungry") the
    # corpus's own Minionese instead of an invented word.  A word several entries
    # would answer with ("you") is left alone rather than resolved by chance, so a
    # word cannot mean one thing alone and another inside a phrase.
    #
    # The Minionese side needs no keys here: it is the vocabulary, and translate()
    # keeps vocabulary words as they are, so a Minionese word or phrase already
    # comes out unchanged (checked in the selftest).  Keying it too would be
    # entries nothing can reach differently - measured, not assumed.
    if index_words:
        owners = collections.defaultdict(set)
        for i, (_minion, other_side) in enumerate(entries):
            for word in _clean_key(other_side).split():
                if word not in _FUNCTION:
                    owners[word].add(i)
        for word, where in owners.items():
            if len(where) == 1:
                table.setdefault(word, entries[next(iter(where))][0])
    return table, vocab, starts, pairs, entries


class _CharModel:
    """An order-3 character model, backed off to order 2.

    Trained on the corpus's Minionese words, it answers the question the table
    cannot: what does a word the corpus has never seen sound like in Minionese?
    Backing off matters because the corpus is small — an unseen prefix of three
    characters still has a usable distribution over its last two.
    """

    ORDERS = (3, 2)
    # The corpus is small, so an unconstrained walk produces twelve characters of
    # mush ("bonononjoutt").  A Minion word is short and ends open.
    MAX_LEN = 8
    MIN_LEN = 4
    OPEN_ENDINGS = "aeiounysko"

    def __init__(self, words):
        self.counts = {n: collections.defaultdict(collections.Counter) for n in self.ORDERS}
        self.alphabet = set()
        for word in words:
            clean = "".join(c for c in word.lower() if c.isalpha())
            if not clean:
                continue
            self.alphabet.update(clean)
            padded = "^" + clean + "$"
            for order in self.ORDERS:
                for i in range(len(padded) - order):
                    self.counts[order][padded[i:i + order - 1]][padded[i + order - 1]] += 1

    def _sample(self, rng, context, order):
        counts = self.counts[order].get(context[-order + 1:])
        if not counts:
            return None
        letters, weights = zip(*counts.items())
        return rng.choices(letters, weights=weights, k=1)[0]

    def word(self, seed_text, tries=40):
        """A Minionese-looking word, the same one every time for the same seed."""
        rng = random.Random(int(hashlib.sha1(seed_text.encode("utf-8")).hexdigest()[:12], 16))
        best = ""
        for _ in range(tries):
            out, context = "", "^"
            while len(out) < self.MAX_LEN:
                nxt = None
                for order in self.ORDERS:
                    nxt = self._sample(rng, context, order)
                    if nxt is not None:
                        break
                if nxt is None or nxt == "$":
                    break
                out += nxt
                context += nxt
            if (self.MIN_LEN <= len(out) <= self.MAX_LEN and any(c in "aeiou" for c in out)
                    and out[-1] in self.OPEN_ENDINGS):
                return out
            if self.MIN_LEN <= len(out) <= self.MAX_LEN and len(out) > len(best):
                best = out
        return best or "banana"          # never return nothing: the corpus's word


class Minionese:
    """The trained language, ready to speak."""

    def __init__(self, text, english=""):
        self.table, self.vocab, self.starts, self.pairs, self.entries = _learn(text)
        # self.entries is the parsed corpus - (Minionese side, taught side) per
        # entry, in corpus order - which is what lets the selftest prove every
        # entry is reachable from its own taught words instead of only proving
        # that the keys it built round-trip.
        #
        # The English column is the other half of the corpus's own meanings: part
        # of it is taught in English (``Bello - Hello``) but part only in French
        # (``Moka : S'il vous plaît``), which left an English speaker unable to
        # reach those meanings at all - measured, "please" and "you're welcome"
        # were invented while the corpus plainly teaches both.  It is trained the
        # same way and merged with setdefault, so the corpus wins any key it
        # already taught, and it contributes no words of its own and no openings:
        # its Minionese side is the corpus's, and the openings stay the corpus's.
        self.english = []
        if english:
            column, words, _starts, _pairs, self.english = _learn(english, index_words=False)
            for key, word in column.items():
                self.table.setdefault(key, word)
            self.vocab.update(words)
        self.char = _CharModel(self.vocab)
        self._invented = {}              # English word -> its Minionese, so it is stable
        self._rng = random.Random()

    # ── the pieces ────────────────────────────────────────────────────────────
    @property
    def longest_key(self):
        """The longest entry, in words.

        The matcher has to be able to reach the longest key: a fixed cap of four
        silently left every longer entry unreachable, which the round-trip check
        found ("Signal sonore de batterie faible", "Sous-vêtement utilisé...").
        """
        return max((len(k.split()) for k in self.table), default=1)

    def opening(self, rng=None):
        """The word to start a sentence with, sampled from the corpus's own."""
        counter = self.starts or collections.Counter(self.vocab)
        words, weights = zip(*counter.items())
        return (rng or self._rng).choices(words, weights=weights, k=1)[0].capitalize()

    def _for_word(self, word):
        key = _clean_key(word)
        if key in self.table:
            return self.table[key]
        if word.lower() in self._invented:
            return self._invented[word.lower()]
        if word.isdigit() and key in self.table:
            return self.table[key]
        made = self.char.word(key)
        self._invented[word.lower()] = made
        return made

    # ── the sentence ──────────────────────────────────────────────────────────
    def translate(self, text):
        """English (or French) in, Minionese out, keeping the corpus's word order."""
        text = (text or "").strip()
        if not text:
            return ""
        tokens = _TOKEN.findall(text)
        out = []
        i = 0
        while i < len(tokens):
            if not tokens[i][0].isalnum():
                out.append(tokens[i])                     # punctuation is kept
                i += 1
                continue
            placed = False
            for n in range(self.longest_key, 0, -1):
                window = tokens[i:i + n]
                if len(window) < n or not all(w[:1].isalpha() for w in window):
                    continue
                key = _clean_key(" ".join(window))
                # A one-word key that is already Minionese is left alone.  The
                # corpus's own words are the vocabulary, and an English key that
                # happens to be one of them would re-translate them: "kiss" is
                # taught as "Kiss kiss", and without this the Minionese phrase
                # "Kiss kiss" came back out as "Kiss kiss Kiss kiss".
                if key in self.table and not (n == 1 and key in self.vocab):
                    out.append(self.table[key])
                    i += n
                    placed = True
                    break
            if placed:
                continue
            word = tokens[i]
            i += 1
            if word.isdigit():
                # The corpus teaches the numerals from one to ten, English for the
                # first three and French for the rest ("Ju : Dix"), so a digit is
                # resolved through either language's word for it.
                numeral = ""
                for name in _NUMBER_WORDS.get(word, ()):
                    numeral = self.table.get(name, "")
                    if numeral:
                        break
                out.append(numeral or self.char.word(word))
            elif word.lower() in _DROP:
                continue
            elif word.lower() in self.vocab:
                out.append(word)
            else:
                out.append(self._for_word(word))
        return _tidy(out)

    def speak(self, text, rng=None):
        """The whole line the robot says: an opening word, then the sentence."""
        body = self.translate(text)
        if not body:
            return ""
        opening = self.opening(rng)
        body = body[0].lower() + body[1:] if len(body) > 1 else body
        return "%s %s" % (opening, body) if body else opening


def _tidy(pieces):
    """Join tokens, keeping the punctuation the caller wrote where it belongs."""
    out = ""
    for piece in pieces:
        if piece in ",.!?;:…":
            out = out.rstrip() + piece
        else:
            out = (out + " " + piece) if out else piece
    out = re.sub(r"\s+", " ", out).strip()
    return out[:1].upper() + out[1:] if out else out


_NUMBER_WORDS = {
    "1": ("one", "un"), "2": ("two", "deux"), "3": ("three", "trois"),
    "4": ("four", "quatre"), "5": ("five", "cinq"), "6": ("six", "six"),
    "7": ("seven", "sept"), "8": ("eight", "huit"), "9": ("nine", "neuf"),
    "10": ("ten", "dix"),
}


def load(path=CORPUS_FILE, english=ENGLISH_FILE):
    """Train from the corpus file and its English column.

    Returns None if the corpus is not there (the robot then speaks plain text).
    A missing English column is not fatal: the language is then only as reachable
    as the corpus's own translation sides.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError:
        return None
    try:
        with open(english, encoding="utf-8", errors="replace") as f:
            column = f.read()
    except OSError:
        column = ""
    return Minionese(text, column)


LANGUAGE = load()


def speak(text, rng=None):
    """The Minionese line for `text`, or the text itself if there is no corpus."""
    if not text or not text.strip():
        return ""
    if LANGUAGE is None:
        return text.strip()
    return LANGUAGE.speak(text, rng=rng)


def translate(text):
    """Minionese for `text` without the opening word."""
    if not text or not text.strip():
        return ""
    if LANGUAGE is None:
        return text.strip()
    return LANGUAGE.translate(text)
