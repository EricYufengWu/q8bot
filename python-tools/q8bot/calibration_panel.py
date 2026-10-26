"""The leg picker and the calibration view, drawn into the preview area.

The mounting diagram is generated from the same solver the robot is commanded
with, so the picture and the held pose cannot drift apart.
"""

import pygame

import eth_theme as eth
import leg_geometry
from leg_calibration import CONFIRM, FAILED, HOLD, MOVING, PICK
from leg_designs import design_for
from pose_preview import PREVIEW_SIZE

PANEL_SIZE = PREVIEW_SIZE


class CalibrationPanel:
    def __init__(self):
        self.fonts = {size: pygame.font.Font(None, size) for size in (17, 18, 20, 22, 23, 26, 44)}

    def _text(self, surface, text, size, color, xy):
        surface.blit(self.fonts[size].render(text, True, color), xy)

    def render(self, calibration):
        surface = pygame.Surface(PANEL_SIZE)
        surface.fill(eth.PANEL)
        if calibration.stage == PICK:
            self._render_picker(surface, calibration)
        else:
            self._render_calibration(surface, calibration)
        return surface

    # --- picker ----------------------------------------------------------
    def _render_picker(self, surface, calibration):
        self._text(surface, 'CHANGE LEGS', 18, eth.ACCENT, (28, 23))
        self._text(surface, 'Leg design', 44, eth.TEXT_PRIMARY, (28, 47))
        self._text(surface, f'Fitted now: {calibration.fitted().title}', 22,
                   eth.TEXT_SECONDARY, (28, 100))
        for index, key in enumerate(calibration.ORDER):
            design = design_for(key)
            y = 142 + index * 80
            selected = index == calibration.cursor
            fitted = key == calibration.fitted_key
            pygame.draw.rect(surface, eth.CHIP if selected else eth.CHIP_DIM,
                             (28, y, 744, 68), border_radius=6)
            if selected:
                pygame.draw.rect(surface, eth.ACCENT_FILL, (28, y, 5, 68),
                                 border_top_left_radius=6, border_bottom_left_radius=6)
            title_color = eth.TEXT_PRIMARY if design.available else eth.TEXT_MUTED
            self._text(surface, design.title, 26, title_color, (48, y + 13))
            self._text(surface, design.subtitle, 20, eth.TEXT_MUTED, (48, y + 40))
            if not design.available:
                self._badge(surface, 'NOT AVAILABLE', eth.POSE_ACCENT, (604, y + 13))
            elif fitted:
                self._badge(surface, 'FITTED', eth.ACCENT, (664, y + 13))
        self._text(surface, 'Calibration drives every joint to that design\'s mounting pose.',
                   18, eth.TEXT_MUTED, (28, 438))

    def _badge(self, surface, text, color, xy):
        width = self.fonts[17].size(text)[0] + 16
        pygame.draw.rect(surface, eth.PANEL, (xy[0], xy[1], width, 22), border_radius=4)
        pygame.draw.rect(surface, color, (xy[0], xy[1], width, 22), 1, border_radius=4)
        self._text(surface, text, 17, color, (xy[0] + 8, xy[1] + 6))

    # --- calibration -----------------------------------------------------
    def _render_calibration(self, surface, calibration):
        design = calibration.chosen()
        stage = calibration.stage
        heading = {CONFIRM: 'READY TO CALIBRATE', MOVING: 'MOVING',
                   HOLD: 'HOLDING MOUNTING POSE', FAILED: 'CALIBRATION FAILED'}[stage]
        accent = eth.LOG_ERROR if stage == FAILED else (
            eth.POSE_ACCENT if stage == HOLD else eth.ACCENT)
        self._text(surface, heading, 18, accent, (28, 23))
        self._text(surface, design.title, 44, eth.TEXT_PRIMARY, (28, 47))
        q1, q2 = design.mount_pose
        t1, t2 = design.mount_ticks()
        self._text(surface, f'Mounting pose  {q1:.3f} deg / {q2:.3f} deg    ticks {t1} / {t2}',
                   20, eth.TEXT_SECONDARY, (28, 100))

        self._draw_linkage(surface, design, (28, 130, 330, 300), accent)

        x = 396
        for index, step in enumerate(design.steps):
            y = 138 + index * 52
            done = stage in (HOLD, MOVING) and index < 2
            colour = eth.TEXT_MUTED if done else eth.TEXT_SECONDARY
            pygame.draw.circle(surface, accent if not done else eth.CHIP, (x + 8, y + 8), 7)
            self._text(surface, str(index + 1), 17,
                       eth.TEXT_ON_ACCENT if not done else eth.TEXT_MUTED, (x + 4, y + 2))
            self._wrap(surface, step, 20, colour, (x + 26, y - 2), 350)

        if design.notes:
            self._text(surface, design.notes[0], 17, eth.TEXT_MUTED, (28, 446))

    def _wrap(self, surface, text, size, color, xy, width):
        font = self.fonts[size]
        line, y = '', xy[1]
        for word in text.split():
            trial = f'{line} {word}'.strip()
            if font.size(trial)[0] > width and line:
                self._text(surface, line, size, color, (xy[0], y))
                line, y = word, y + size - 2
            else:
                line = trial
        if line:
            self._text(surface, line, size, color, (xy[0], y))

    def _draw_linkage(self, surface, design, box, accent):
        """Scale the design's own pivots into the box and draw the linkage."""
        bx, by, bw, bh = box
        pygame.draw.rect(surface, eth.BACKGROUND, box, border_radius=6)
        try:
            solver = design.solver()
        except ValueError:
            return
        pivots = leg_geometry.pivots(solver, *design.mount_pose)
        xs = [p[0] for p in pivots.values()]
        ys = [p[1] for p in pivots.values()]
        pad = 34
        span = max(max(xs) - min(xs), max(ys) - min(ys)) or 1
        scale = (min(bw, bh) - 2 * pad) / span
        ox = bx + bw / 2 - (min(xs) + max(xs)) / 2 * scale
        oy = by + bh / 2 - (min(ys) + max(ys)) / 2 * scale

        def at(name):
            x, y = pivots[name]
            return round(ox + x * scale), round(oy + y * scale)

        style = {leg_geometry.CRANK: (5, accent), leg_geometry.PUSHROD: (4, eth.SHANK),
                 leg_geometry.BONE: (7, eth.CHASSIS_EDGE), leg_geometry.FRAME: (3, eth.CHIP)}
        for a, b, role in leg_geometry.links(pivots):
            width, colour = style[role]
            pygame.draw.line(surface, eth.OUTLINE, at(a), at(b), width + 4)
            pygame.draw.line(surface, colour, at(a), at(b), width)
        motors = leg_geometry.motor_pivots()
        for name in pivots:
            centre = at(name)
            pygame.draw.circle(surface, eth.OUTLINE, centre, 6)
            pygame.draw.circle(surface, accent if name in motors else eth.JOINT_CORE, centre, 4)
        for name in motors:
            cx, cy = at(name)
            self._text(surface, 'servo', 17, eth.TEXT_MUTED, (cx - 16, cy - 22))
        fx, fy = at(leg_geometry.foot_pivot())
        self._text(surface, 'foot', 17, eth.TEXT_MUTED, (fx - 12, fy + 10))
