"""Drawing of the leg-detection screens, in the preview area."""

import pygame

import eth_theme as eth
from leg_detection import LEGS
from pose_preview import PREVIEW_SIZE

JOINTS = ('elbow', 'shoulder')


def diagnosis_body(diagnosis):
    return f'{diagnosis.detail[0].upper()}{diagnosis.detail[1:]}. {diagnosis.action}'


class StartupPanel:
    def __init__(self):
        self.fonts = {size: pygame.font.Font(None, size) for size in (17, 18, 20, 22, 26, 44)}

    def render_card(self, heading, title, body, accent=eth.LOG_ERROR):
        """A plain message in the preview area: a stopped stand-up, say."""
        surface = pygame.Surface(PREVIEW_SIZE)
        surface.fill(eth.PANEL)
        self._text(surface, heading, 18, accent, (28, 23))
        self._text(surface, title, 44, eth.TEXT_PRIMARY, (28, 47))
        self._wrap(surface, body, 22, eth.TEXT_SECONDARY, (28, 100))
        return surface

    def _text(self, surface, text, size, color, xy):
        surface.blit(self.fonts[size].render(text, True, color), xy)

    def _wrap(self, surface, text, size, color, xy, width=744):
        font, line, y = self.fonts[size], '', xy[1]
        for word in text.split():
            trial = f'{line} {word}'.strip()
            if font.size(trial)[0] > width and line:
                self._text(surface, line, size, color, (xy[0], y))
                line, y = word, y + size - 1
            else:
                line = trial
        if line:
            self._text(surface, line, size, color, (xy[0], y))
        return y + size

    def render(self, flow):
        surface = pygame.Surface(PREVIEW_SIZE)
        surface.fill(eth.PANEL)
        detector = flow.detector
        heading, title, body, accent = {
            'connecting': ('LEG DETECTION', 'Contacting the robot',
                           'Asking for the joint angles. Nothing moves.', eth.ACCENT),
            'lift': ('LEG DETECTION', 'Lift the robot',
                     'Hold or support it so all four legs hang free and nothing touches '
                     'them. Each joint is then swept gently, at low output, to its stop and '
                     'back, about half a minute in all. Sweeps never come near a point '
                     'where a leg linkage could fold.', eth.ACCENT),
            'detecting': ('DETECTING', 'Sweeping the joints', detector.progress(), eth.POSE_ACCENT),
            'result': ('RESULT', 'Legs not recognised', '', eth.ACCENT),
            'choose': ('CHOOSE LEGS', 'Which legs are fitted?',
                       'The robot will be driven with this design. Choose carefully: the '
                       'wrong one can drive a leg into the body.', eth.ACCENT),
            'failed': ('STOPPED', 'Detection stopped', detector.error or '', eth.LOG_ERROR),
            'no_reply': ('NO ANSWER', 'The robot did not answer',
                         'Its firmware may predate leg detection, or it is switched off. '
                         'Flash the current q8bot_robot firmware, or choose the legs manually.',
                         eth.LOG_WARNING),
            'done': ('', '', '', eth.ACCENT),
        }[flow.stage]
        result = detector.result
        if flow.stage == 'result' and result and result['design']:
            title = f'Detected: {next(d.title for d in flow.designs if d.key == result["design"])}'
        if flow.stage == 'failed' and detector.diagnosis:
            title, body = detector.diagnosis.title, diagnosis_body(detector.diagnosis)
        self._text(surface, heading, 18, accent, (28, 23))
        self._text(surface, title, 44, eth.TEXT_PRIMARY, (28, 47))
        y = 100
        if body:
            y = self._wrap(surface, body, 22, eth.TEXT_SECONDARY, (28, y)) + 8
        if flow.stage == 'failed' and detector.restore_failed:
            y = self._wrap(surface, 'Torque-off was not confirmed on every servo. Switch the '
                                    'battery off before touching the legs.', 22, eth.LOG_ERROR, (28, y)) + 8

        if flow.stage in ('lift', 'connecting') and detector.telemetry:
            self._angles(surface, detector, y + 4)
        elif flow.stage in ('detecting', 'result', 'failed'):
            self._sweeps(surface, detector, y + 4)
        if flow.stage == 'result' and result:
            self._wrap(surface, f'{result["reason"].capitalize()}.', 20, eth.TEXT_MUTED, (28, 300))
            self._designs(surface, flow, 330, result)
        elif flow.stage == 'choose':
            self._designs(surface, flow, max(y + 8, 200), None)
        if flow.stage == 'lift' and detector.problems:
            self._wrap(surface, 'Not ready: ' + '; '.join(detector.problems) + '.', 20,
                       eth.LOG_WARNING, (28, 440))
        return surface

    def _angles(self, surface, detector, y):
        servos = detector.telemetry['servos']
        for leg in range(4):
            x = 28 + leg * 186
            self._text(surface, LEGS[leg], 20, eth.ACCENT, (x, y))
            for j in range(2):
                servo = servos.get(11 + leg * 2 + j, {})
                text = f'{JOINTS[j]} {servo["deg"]:7.1f}' if servo.get('responding') else f'{JOINTS[j]}  --'
                self._text(surface, text, 20, eth.TEXT_SECONDARY, (x, y + 24 + j * 22))

    def _sweeps(self, surface, detector, y):
        """One cell per joint and direction: contact angle, free, or not swept."""
        for leg in range(4):
            x = 28 + leg * 186
            self._text(surface, LEGS[leg], 20, eth.ACCENT, (x, y))
            row = 0
            for part in ('q1', 'q2'):
                for sign in ('+', '-'):
                    entry = detector.measurement.get(f'{LEGS[leg]}.{part}{sign}')
                    name = f'{JOINTS[0 if part == "q1" else 1][:5]} {sign}'
                    if entry is None:
                        text, colour = f'{name}  ...', eth.TEXT_MUTED
                    elif entry['kind'] == 'contact':
                        text, colour = f'{name}  stop {entry["stop"]:6.1f}', eth.TEXT_SECONDARY
                    elif entry['kind'] == 'free':
                        text, colour = f'{name}  free {entry["bound"]:4.0f}', eth.TEXT_MUTED
                    else:
                        text, colour = f'{name}  {entry["kind"]}', eth.TEXT_MUTED
                    self._text(surface, text, 18, colour, (x, y + 24 + row * 20))
                    row += 1

    def _designs(self, surface, flow, y, result):
        for index, design in enumerate(flow.designs):
            top = y + index * 44
            selected = index == flow.cursor
            pygame.draw.rect(surface, eth.CHIP if selected else eth.CHIP_DIM,
                             (28, top, 744, 38), border_radius=6)
            if selected:
                pygame.draw.rect(surface, eth.ACCENT_FILL, (28, top, 5, 38),
                                 border_top_left_radius=6, border_bottom_left_radius=6)
            self._text(surface, design.title, 22, eth.TEXT_PRIMARY, (46, top + 11))
            notes = []
            if result:
                if design.key in result['scores']:
                    good, bad = result['scores'][design.key]
                    notes.append(f'{good} stops agree, {bad} disagree')
                else:
                    notes.append('no recording yet')
            if design.key == flow.last:
                notes.append('last used')
            self._text(surface, '   '.join(notes), 18, eth.TEXT_MUTED, (300, top + 13))
