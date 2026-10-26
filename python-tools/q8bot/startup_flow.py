"""The leg-detection screens shown before the robot is allowed to move.

Nothing here commands a joint directly: sweeps go through LegDetector, which
asks the robot firmware for bounded, low-output probes. The flow only decides
what the user is asked next and which design the session starts with.
"""

import pygame

from leg_designs import selectable
from leg_detection import load_last_design, save_last_design


class StartupFlow:
    """Stages: connecting, lift, detecting, result, choose, failed, no_reply, done."""

    def __init__(self, detector, log):
        self.detector = detector
        self.log = log
        self.stage = 'connecting'
        self.chosen = None
        self.designs = [design for design in selectable() if design.available]
        self.last = load_last_design()
        self.cursor = self._index(self.last)
        self.pending_record = None     # design key awaiting a second R

    @property
    def active(self):
        return self.stage != 'done'

    def highlighted(self):
        return self.designs[self.cursor]

    def begin(self):
        self.stage = 'connecting'
        self.detector.request_telemetry()

    # --- input -----------------------------------------------------------
    def handle(self, events, replies):
        """Feed replies and keys. Returns True when Esc should quit the program."""
        for reply in replies:
            self.detector.feed(reply)
        quit_requested = False
        for event in events:
            if event.type != pygame.KEYDOWN:
                continue
            if event.key == pygame.K_ESCAPE:
                if self.stage == 'detecting':
                    # Esc during a sweep stops the sweep, not the program.
                    self.detector.abort()
                    self.log.info('Leg detection cancelled; the robot releases the legs.')
                    self.begin()
                else:
                    quit_requested = True
            elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self._enter()
            elif event.key in (pygame.K_UP, pygame.K_LEFT) and self.stage in ('result', 'choose'):
                self.cursor = (self.cursor - 1) % len(self.designs)
                self.pending_record = None
            elif event.key in (pygame.K_DOWN, pygame.K_RIGHT) and self.stage in ('result', 'choose'):
                self.cursor = (self.cursor + 1) % len(self.designs)
                self.pending_record = None
            elif event.key == pygame.K_m and self.stage in ('lift', 'no_reply', 'failed', 'result'):
                self.stage = 'choose'
            elif event.key == pygame.K_r and self.stage == 'result':
                # Two presses on the same highlighted design: a recording under
                # the wrong design would make every later detection ambiguous.
                design = self.highlighted()
                if self.pending_record != design.key:
                    self.pending_record = design.key
                    self.log.info(f'Press R again to record these stops as {design.title}.')
                else:
                    self.pending_record = None
                    count = self.detector.record(design.key)
                    self.log.info(f'Recorded these stops as {design.title} ({count} recording(s)).')
        return quit_requested

    def _enter(self):
        if self.stage == 'lift':
            if self.detector.problems:
                self.begin()                      # re-read after the user fixed it
            elif self.detector.start():
                self.stage = 'detecting'
                self.log.info('Detecting legs: keep the robot lifted, hands clear.')
        elif self.stage in ('no_reply', 'failed'):
            self.begin()
        elif self.stage in ('result', 'choose'):
            design = self.highlighted()
            found = (self.detector.result or {}).get('design')
            if self.stage == 'result' and found and found != design.key:
                self.log.warning(f'Using {design.title} although the stops matched {found}.')
            self._choose(design)

    def _choose(self, design):
        self.chosen = design
        save_last_design(design.key)
        self.stage = 'done'

    def update(self):
        self.detector.update()
        stage = self.detector.stage
        if self.stage == 'connecting':
            if stage == 'ready':
                self.stage = 'lift'
            elif stage == 'no_reply':
                self.stage = 'no_reply'
        elif self.stage == 'detecting':
            if stage == 'done':
                self.stage = 'result'
                found = self.detector.result['design']
                if found:
                    self.cursor = self._index(found)
                    self.log.info(f'Detected {found} legs.')
                else:
                    self.log.info(f'Legs not recognised: {self.detector.result["reason"]}.')
            elif stage == 'failed':
                self.stage = 'failed'
                self.log.error(f'Leg detection stopped: {self.detector.error}.')

    def _index(self, key):
        for index, design in enumerate(self.designs):
            if design.key == key:
                return index
        return 0

    # --- presentation ----------------------------------------------------
    def label(self):
        return {
            'connecting': 'Contacting the robot...  |  Esc: Exit',
            'lift': 'Enter: detect legs   M: choose manually   Esc: Exit',
            'detecting': 'Detecting legs - keep the robot lifted  |  Esc: stop',
            'result': ('R again: record as ' + self.highlighted().title if self.pending_record
                       else 'Up/Down: design   Enter: use it   R: record as its stops   M: list   Esc: Exit'),
            'choose': 'Up/Down: design   Enter: use it   Esc: Exit',
            'failed': 'Enter: try again   M: choose manually   Esc: Exit',
            'no_reply': 'Enter: try again   M: choose manually   Esc: Exit',
            'done': '',
        }[self.stage]
