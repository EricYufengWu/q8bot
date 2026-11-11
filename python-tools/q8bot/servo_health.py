"""Why a servo action failed: communication, torque, power or something else.

The robot answers a telemetry request with, per servo (IDs 11-18 legs, 19
head): whether it answered, its torque state, position, current and Hardware
Error Status. That is enough to tell the usual causes apart:

    communication   a servo does not answer, or dropped out during a move
    torque          a servo shut down on overload, or sits far short of its goal
    power           a servo flagged its input voltage
    heat            a servo overheated
    servo           encoder or electrical-shock fault inside a servo
    pose            the legs are in a pose the motion cannot start from
    disturbed       a held joint was pushed out of place

Hardware errors latch until the servo loses power, so once their cause is
dealt with the remedy is always a power cycle.
"""

LEGS = ('FL', 'FR', 'BL', 'BR')
JOINTS = ('elbow', 'shoulder')       # q1, q2
LEG_IDS = tuple(range(11, 19))
HEAD_ID = 19
LAG_TOLERANCE = 12.0                 # deg a loaded joint may sit short of its goal

# Hardware Error Status bits of the XL330, most telling first.
HARDWARE_ERRORS = (
    (5, 'torque', 'overload', 'it was asked for more torque than it has: a blocked, '
                              'caught or overloaded joint'),
    (0, 'power', 'an input voltage error', 'its supply left the allowed range: check the '
                                           'battery charge and the power wiring'),
    (2, 'heat', 'overheating', 'let it cool before running it again'),
    (4, 'servo', 'an electrical shock error', 'the servo itself may be damaged'),
    (3, 'servo', 'a motor encoder error', 'the servo itself may be damaged'),
)
LATCHED = 'The error stays latched until the servo loses power: switch the battery off and on.'
INTERMITTENT = ('Every servo answers and reports no fault now, so the link drops out now and '
                'then: reseat the servo cables and connectors, above all where the legs flex, '
                'and check the battery.')

TITLES = {
    'communication': 'Communication problem',
    'torque': 'Not enough torque',
    'power': 'Power problem',
    'heat': 'Servo overheated',
    'servo': 'Servo hardware fault',
    'pose': 'Start pose not usable',
    'disturbed': 'Leg disturbed',
    'state': 'Robot not ready',
}


class Diagnosis:
    """One cause: a category, what was seen, and what to do about it."""

    def __init__(self, category, detail, action, servos=()):
        self.category, self.detail, self.action = category, detail, action
        self.servos = tuple(servos)

    @property
    def title(self):
        return TITLES[self.category]

    def text(self):
        return f'{self.title}: {self.detail}. {self.action}'

    def __repr__(self):
        return f'Diagnosis({self.text()!r})'


def servo_name(servo_id):
    if servo_id == HEAD_ID:
        return f'head (ID {HEAD_ID})'
    leg, joint = divmod(servo_id - 11, 2)
    return f'{LEGS[leg]} {JOINTS[joint]} (ID {servo_id})'


def _names(ids):
    return ', '.join(servo_name(i) for i in ids)


def _wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def no_answer():
    return Diagnosis('communication', 'the robot did not answer',
                     'Check that the robot is switched on and the controller dongle is plugged in.')


def intermittent(detail):
    return Diagnosis('communication', detail, INTERMITTENT)


def check(telemetry, expected=None, ids=LEG_IDS, tolerance=LAG_TOLERANCE):
    """The most serious problem these readings show, or None if they show none.

    ``expected`` (the 8 leg joint angles last sent, FL FR BL BR) adds a check
    that every leg servo is within ``tolerance`` of its goal.
    """
    if telemetry is None:
        return no_answer()
    servos = telemetry['servos']
    silent = [i for i in ids if not servos.get(i, {}).get('responding')]
    if silent:
        return Diagnosis('communication', f'no answer from {_names(silent)}',
                         'Check the cable into that servo and the connectors either side of it '
                         'in the chain; a battery dip can also drop servos.', silent)
    for bit, category, name, cause in HARDWARE_ERRORS:
        flagged = [i for i in ids if servos[i].get('error', 0) & (1 << bit)]
        if flagged:
            return Diagnosis(category, f'{_names(flagged)} reports {name}',
                             f'{cause[0].upper()}{cause[1:]}. {LATCHED}', flagged)
    faulted = [i for i in ids if servos[i].get('fault')]
    if faulted:
        return Diagnosis('servo', f'{_names(faulted)} reports hardware error '
                                  f'{servos[faulted[0]].get("error")}', LATCHED, faulted)
    if expected is not None:
        lags = sorted(((abs(_wrap(servos[11 + i]['deg'] - goal)), 11 + i)
                       for i, goal in enumerate(expected) if 11 + i in ids), reverse=True)
        lags = [(lag, i) for lag, i in lags if lag > tolerance]
        if lags:
            lag, worst = lags[0]
            current = servos[worst].get('current')
            detail = f'{servo_name(worst)} is {lag:.0f} deg short of its goal'
            if current is not None:
                detail += f', drawing {abs(current)} mA'
            if len(lags) > 1:
                detail += f'; {len(lags) - 1} more joint(s) short too'
            return Diagnosis('torque', detail,
                             'The joint could not get there: too little torque for the load, or '
                             'the leg is blocked or caught.', [i for _, i in lags])
    return None


# Leg-probe outcomes (espnow.STATUSES) by what they mean when the servos look
# healthy afterwards.
PROBE_CATEGORIES = {1: 'state', 2: 'state', 3: 'state', 4: 'servo', 5: 'communication',
                    6: 'servo', 7: 'communication', 8: 'disturbed', 10: 'communication',
                    11: 'communication'}
PROBE_ACTIONS = {
    'communication': INTERMITTENT,
    'disturbed': 'Keep hands and objects clear of the legs while they are swept.',
    'state': 'Release the legs (torque off), then try again.',
    'servo': LATCHED,
}


def after_probe(status, status_text, telemetry, ids=LEG_IDS):
    """Diagnose a failed sweep from the readings taken right after it."""
    found = check(telemetry, ids=ids)
    if found:
        return found
    category = PROBE_CATEGORIES.get(status, 'communication')
    return Diagnosis(category, status_text, PROBE_ACTIONS[category])
