# BatteryGuard — park on a sagging pack before the cutoff parks the Pi.

# Measured on this robot: the pack sat at 10.45 V resting after a charge and
# sagged to 9.77 V while cruising; an earlier session drove the pack to the
# undervoltage cutoff at ~9.0 V, which killed the Pi mid-drive and left the
# robot's state unknown until someone walked over.  The cutoff is not a
# shutdown anyone can act on — the guard's job is to park and say so while
# there is still pack left to park with.

LOW_SAG_V = 9.80           # sustained sag under load: finish and park
CRIT_SAG_V = 9.25          # any single reading here: park immediately
REST_OK_V = 10.60          # resting pack above this is healthy again
SUSTAIN_S = 6.0            # how long LOW must hold before it counts
RECOVER_S = 30.0           # how long healthy must hold before re-arm

ANNOUNCE = "Bee do bee do bee do! Battery low. Banana time - me go home!"

def battery_guard_announce():
    """What the robot says when the guard parks it.

    In its own voice the corpus already teaches for alarms: 'Bee do bee do
    bee do' is Minionese for fire/alarm, 'banana' for food/energy, and the
    rest carries the meaning in English — the voice path translates what it
    can and speaks the sentence whole.
    """
    return ANNOUNCE


class BatteryGuard:
    """Sag judgment for one battery feed, latching until the pack recovers.

    Pure: no hardware, no threads.  `update(voltage, now)` returns
    'ok' | 'low' | 'critical'.  A *single* critical reading parks at once —
    a brownout this deep has no grace period — while low must persist
    SUSTAIN_S, so a hill or a reverse burst cannot park a healthy robot.
    The state latches: 'low'/'critical' until REST_OK_V has held
    RECOVER_S, because a pack that sags will sag again the moment the
    wheels spin.
    """

    def __init__(self, low_v=LOW_SAG_V, crit_v=CRIT_SAG_V,
                 rest_v=REST_OK_V, sustain_s=SUSTAIN_S, recover_s=RECOVER_S):
        self._low_v, self._crit_v, self._rest_v = low_v, crit_v, rest_v
        self._sustain_s, self._recover_s = sustain_s, recover_s
        self.low_sag_since = None    # when the current low-sag streak began
        self.rest_since = None       # when the current healthy streak began
        self.latched = False         # parked: stays until recovery
        self.state = 'ok'

    def update(self, voltage, now=None):
        if not isinstance(voltage, (int, float)) or voltage <= 0:
            return self.state        # a missing reading is not news either way
        now = time.time() if now is None else now

        if self.latched:
            if voltage >= self._rest_v:
                self.rest_since = self.rest_since or now
                if now - self.rest_since >= self._recover_s:
                    self.latched = False
                    self.state = 'ok'
                    self.low_sag_since = None
                    self.rest_since = None
            else:
                self.rest_since = None
            return self.state

        if voltage <= self._crit_v:
            self.latched = True
            self.state = 'critical'
            self.low_sag_since = None
            return self.state

        if voltage < self._low_v:
            self.low_sag_since = self.low_sag_since or now
            if now - self.low_sag_since >= self._sustain_s:
                self.latched = True
                self.state = 'low'
            return self.state

        # healthy reading: both streaks reset
        self.low_sag_since = None
        self.rest_since = None
        self.state = 'ok'
        return self.state
