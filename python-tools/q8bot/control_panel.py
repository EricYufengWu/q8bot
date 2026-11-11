"""Keyboard/joystick reference and the selected gait's illustration."""
from pathlib import Path
import pygame
import eth_theme as eth
from calibration_panel import CalibrationPanel
from control_config import KEYBOARD_MAPPING
from leg_designs import DEFAULT_DESIGN, design_for
from pose_preview import DESCRIPTIONS, PREVIEW_SIZE, make_preview, preview_key
from startup_panel import StartupPanel, diagnosis_body

WINDOW_SIZE = (1280, 800)


class ControlPanel:
    def __init__(self, assets_dir, use_joystick=False, joystick_mapping=None,
                 design_key=DEFAULT_DESIGN):
        self.fonts = {size: pygame.font.Font(None, size) for size in (18, 20, 23, 26, 32)}
        self.assets_root = Path(assets_dir)
        self._sets = {}
        self.design_key = design_key
        self.previews = self._load(design_key)
        self.calibration_view = CalibrationPanel()
        self.startup_view = StartupPanel()
        self._joystick, self._mapping = use_joystick, joystick_mapping
        self._standing = {}
        self.controls = self._make_controls(use_joystick, joystick_mapping, design_for(design_key))
        self.startup_controls = self._make_startup_controls()
        self.active_preview = None

    def _load(self, design_key):
        """Pictures for one leg design, cached so a swap back is instant."""
        if design_key in self._sets:
            return self._sets[design_key]
        design = design_for(design_key)
        # Designs keep their pictures in their own folder. Older checkouts put
        # the five-bar set straight in docs/poses, so fall back to that.
        folders = [self.assets_root / design.asset_dir, self.assets_root]
        pictures = {}
        for name in DESCRIPTIONS:
            picture = None
            for folder in folders:
                try:
                    picture = pygame.image.load(str(folder / f'{name.lower()}.png'))
                    break
                except (OSError, pygame.error):
                    continue
            if picture is None:
                # Source checkouts and packaged apps both remain usable if an
                # asset is missing; drawing it costs a moment instead.
                picture = make_preview(name, design)
            pictures[name] = pygame.transform.smoothscale(picture, PREVIEW_SIZE)
        self._sets[design_key] = pictures
        return pictures

    def set_design(self, design_key):
        """Show the fitted design's pictures from now on."""
        self.design_key = design_key
        self.previews = self._load(design_key)
        self.controls = self._make_controls(self._joystick, self._mapping, design_for(design_key))

    def _standing_picture(self, design):
        """The per-leg standing picture, drawn once per design."""
        if design.key not in self._standing:
            self._standing[design.key] = make_preview('STANDING', design)
        return self._standing[design.key]

    def _text(self, surface, text, size, color, xy):
        surface.blit(self.fonts[size].render(text, True, color), xy)

    def _make_controls(self, joystick, mapping, design=None):
        panel = pygame.Surface((398, 564))
        panel.fill(eth.PANEL)
        self._text(panel, 'CONTROLS', 20, eth.ACCENT, (24, 24))
        self._text(panel, 'Joystick + keyboard' if joystick else 'Keyboard', 32, eth.TEXT_PRIMARY, (24, 53))
        walks = design is None or design.supports_gaits
        move = KEYBOARD_MAPPING['movement']
        keys = lambda names: '/'.join(pygame.key.name(move[n]).upper() for n in names)
        head_row = ('LEFT/RIGHT', 'Turn head')
        if not walks:
            # Gaits and the sit/rear/greet poses were built for a single leg
            # model; legs whose hind pair differs get the per-leg stand-up and
            # test trot, the head, and nothing single-model.
            rows = [(keys(('forward', 'backward')), 'Walk (trot)'),
                    (keys(('left', 'right')), 'Turn on the spot'), head_row]
            actions = [('stand', 'Stand up / lie down'), ('switch_gait', 'Slow / fast trot'),
                       ('centre_head', 'Centre head'),
                       ('swap_head', 'Swap head left/right'), ('change_legs', 'Change legs'),
                       ('battery', 'Battery level'), ('exit', 'Exit / release torque')]
        elif joystick:
            rows = [('Stick', 'Move / turn'), head_row]
        else:
            rows = [(keys(('forward', 'left', 'backward', 'right')), 'Move / turn'),
                    (keys(('forward_left', 'forward_right')), 'Forward with turn'), head_row]
        if walks:
            actions = [
                ('sit', 'Sit down / stand up'), ('rear', 'Rear up / lower'),
                ('switch_gait', 'Switch gait'), ('change_legs', 'Change legs'),
                ('reset', 'Reset gait'), ('greet', 'Greet'), ('jump', 'Jump'),
                ('battery', 'Battery level'), ('show_range', 'Show range'),
                ('record', 'Record movement'), ('centre_head', 'Centre head'),
                ('exit', 'Exit / release torque'),
            ]
        for action, description in actions:
            keyboard_only = ('sit', 'rear', 'exit', 'stand', 'centre_head', 'swap_head')
            if (joystick and action not in keyboard_only) and walks:
                button = (mapping or {}).get('actions', {}).get(action)
                if button is None:
                    continue
                key = f'Btn {button}'
            else:
                key = pygame.key.name(KEYBOARD_MAPPING['actions'][action]).upper()
                key = {'ESCAPE': 'ESC'}.get(key, key)
            rows.append((key, description))
        # Tighter rows when the list is long, so the last one clears the hint.
        pitch = min(32, (500 - 109) // max(1, len(rows) - 1))
        for index, (key, description) in enumerate(rows):
            y = 109 + index * pitch
            pose_row = description in ('Sit down / stand up', 'Rear up / lower', 'Stand up / lie down')
            color = eth.POSE_ACCENT if pose_row else eth.TEXT_SECONDARY
            pygame.draw.rect(panel, eth.CHIP, (20, y - 4, 115, 29), border_radius=5)
            self._text(panel, key, 20, color, (29, y + 2))
            self._text(panel, description, 23, color, (147, y + 1))
        if not walks:
            top = 109 + len(rows) * pitch + 12
            self._text(panel, f'{design.title} legs: sitting and rearing', 20, eth.TEXT_MUTED, (24, top))
            self._text(panel, 'are not enabled yet.', 20, eth.TEXT_MUTED, (24, top + 22))
        self._text(panel, 'Click this window to use keyboard controls.', 18, eth.TEXT_MUTED, (24, 537))
        return panel

    def draw(self, window, gait, pose, calibration=None, startup=None, head=None, trot=None):
        self._head(window, head, startup)
        self._trot = trot
        if startup is not None and startup.active:
            return self._draw_startup(window, startup)
        if calibration is not None and calibration.active:
            return self._draw_calibration(window, calibration)
        design = design_for(self.design_key)
        if not design.supports_gaits:
            return self._draw_stand_mode(window, design, pose)
        name = preview_key(gait, pose.active, pose.POSE_NAME)
        self.active_preview = name
        window.blit(self.controls, (20, 158))
        window.blit(self.previews[name], (440, 158))
        label = f'Return gait: {DESCRIPTIONS[gait][0]}' if pose.active else f'Selected gait: {DESCRIPTIONS[gait][0]}'
        self._text(window, label, 23, eth.TEXT_SECONDARY, (444, 652))
        # Selection chips are indicators, matching the P and G keyboard controls.
        for index, key in enumerate(DESCRIPTIONS):
            x, y = 440 + (index % 5) * 161, 680 + (index // 5) * 32
            selected = key == name
            # Solid ETH Blue (bronze for poses) carries white text when selected;
            # the ramp colour stays visible as an edge bar in both states.
            pygame.draw.rect(window, eth.CHIP_ACTIVE[key] if selected else eth.CHIP_DIM,
                             (x, y, 153, 26), border_radius=5)
            pygame.draw.rect(window, eth.GAIT_COLORS[key], (x, y, 4, 26),
                             border_top_left_radius=5, border_bottom_left_radius=5)
            self._text(window, DESCRIPTIONS[key][0], 20,
                       eth.TEXT_ON_ACCENT if selected else eth.TEXT_MUTED, (x + 14, y + 6))
        self._status(window, pose.label())

    def _make_startup_controls(self):
        """Until a design is chosen only these keys do anything."""
        panel = pygame.Surface((398, 564))
        panel.fill(eth.PANEL)
        self._text(panel, 'LEG DETECTION', 20, eth.ACCENT, (24, 24))
        self._text(panel, 'Before anything moves', 32, eth.TEXT_PRIMARY, (24, 53))
        rows = [('ENTER', 'Detect / use design'), ('M', 'Choose manually'),
                ('UP / DOWN', 'Pick a design'), ('R', 'Record these stops'),
                ('ESC', 'Stop sweep / exit')]
        for index, (key, description) in enumerate(rows):
            y = 109 + index * 32
            pygame.draw.rect(panel, eth.CHIP, (20, y - 4, 115, 29), border_radius=5)
            self._text(panel, key, 20, eth.TEXT_SECONDARY, (29, y + 2))
            self._text(panel, description, 23, eth.TEXT_SECONDARY, (147, y + 1))
        notes = ('No torque is applied until a design is', 'chosen. Q8 legs then go to their rest',
                 'pose; biomimetic legs stay released', 'until T stands the robot up.')
        for index, line in enumerate(notes):
            self._text(panel, line, 20, eth.TEXT_MUTED, (24, 290 + index * 22))
        return panel

    def _draw_startup(self, window, startup):
        self.active_preview = 'STARTUP'
        window.blit(self.startup_controls, (20, 158))
        window.blit(self.startup_view.render(startup), (440, 158))
        self._status(window, startup.label())

    def _head(self, window, head, startup):
        """Head angle and its limits, above the picture, outside the startup screens."""
        if head is None or (startup is not None and startup.active):
            return
        colour = eth.TEXT_SECONDARY if head.measured else eth.LOG_WARNING
        self._text(window, head.label(), 20, colour, (444, 128))

    def _draw_stand_mode(self, window, design, pose):
        self.active_preview = 'STANDING'
        window.blit(self.controls, (20, 158))
        if getattr(pose, 'stage', None) == 'failed':
            diagnosis = pose.diagnosis
            card = (self.startup_view.render_card('STOPPED', diagnosis.title, diagnosis_body(diagnosis))
                    if diagnosis else
                    self.startup_view.render_card('STOPPED', 'Checking the servos',
                                                  f'{pose.error[0].upper()}{pose.error[1:]}. Reading '
                                                  'every servo back to find out why...'))
            window.blit(card, (440, 158))
        else:
            window.blit(self._standing_picture(design), (440, 158))
        self._text(window, f'Fitted legs: {design.title}', 23, eth.TEXT_SECONDARY, (444, 652))
        trot = getattr(self, '_trot', None)
        if trot is None:
            note = 'Front and hind legs follow their own models. No trot for these legs.'
        else:
            gait = trot.profile
            spec = f'{gait.stride:g} mm stride, {gait.cycle_s:.2f} s cycle, ~{gait.speed:.0f} mm/s'
            if trot.walking:
                name = {'f': 'forward', 'b': 'backward', 'l': 'turning left', 'r': 'turning right'}[trot.direction]
                note = f'{gait.title.capitalize()} {name}: {spec}. Release the keys to stop.'
            else:
                note = f'{gait.title.capitalize()} ({spec}). W/S walk, A/D turn, G switches gait.'
        self._text(window, note, 20, eth.TEXT_MUTED, (444, 684))
        self._status(window, pose.label())

    def _draw_calibration(self, window, calibration):
        self.active_preview = calibration.POSE_NAME
        window.blit(self.controls, (20, 158))
        window.blit(self.calibration_view.render(calibration), (440, 158))
        self._status(window, calibration.label())

    def _status(self, window, text):
        pygame.draw.rect(window, eth.STATUS_BAR, (0, 754, 1280, 46))
        self._text(window, text, 26, eth.TEXT_ON_ACCENT, (24, 767))
