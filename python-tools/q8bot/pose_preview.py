"""Generate gait illustrations from Q8bot kinematics, with no hardware access.

Pictures are per leg design. The limb shape comes from ``leg_geometry``, which
is the same description the calibration diagram draws and the solvers command,
so an illustration cannot show a linkage the robot will not make.
"""
import math
from pathlib import Path

import pygame
import eth_theme as eth
import leg_geometry
from gait_manager import GaitManager
from leg_designs import DEFAULT_DESIGN, design_for
from squro_gait import STAND_DEPTH, standing_positions

PREVIEW_SIZE = (800, 480)
DESCRIPTIONS = {
    'TROT': ('Trot', 'Diagonal leg pairs move together.'),
    'TROT_HIGH': ('High trot', 'A taller stance with a shorter stride.'),
    'TROT_LOW': ('Low trot', 'A crouched stance with a shorter stride.'),
    'TROT_FAST': ('Fast trot', 'A longer stride and a shorter gait cycle.'),
    'WALK': ('Walk', 'Four evenly spaced leg phases.'),
    'CRAWL': ('Crawl', 'Legs lift in sequence with short steps.'),
    'BOUND': ('Bound', 'Front and rear pairs move in turn.'),
    'PRONK': ('Pronk', 'All four legs move together.'),
    'SITTING': ('Sitting', 'Front extended. Rear crouched. Hold this pose.'),
    'REARING': ('Rearing', 'Rear support stance. Forelegs held still.'),
}
COLORS = eth.GAIT_COLORS
# Pictures rendered on demand rather than shipped as assets. STANDING is drawn
# with each leg's own solver, so SQuRo hind legs appear knee-forward.
EXTRA = {'STANDING': ('Standing', 'Front pads and hind toes on the ground, at walking height.')}


def preview_key(gait, active, pose_name='SITTING'):
    return pose_name if active else gait


def _pose_positions(name, design, leg):
    """The eight joint angles and four body heights this picture shows."""
    if name == 'STANDING':
        positions = standing_positions(design) if design.per_leg else design.mount_positions()
        heights = [solver.contact_depth(*positions[i * 2: i * 2 + 2])
                   if hasattr(solver, 'contact_depth') else solver.fk_solve(*positions[i * 2: i * 2 + 2])[1]
                   for i, solver in enumerate(design.leg_solvers())]
        return positions, heights
    if name == 'SITTING':
        foot_x, front, rear = design.sitting
        heights = [front] * 2 + [rear] * 2
        positions = []
        for height in heights:
            q1, q2, valid = leg.ik_solve(foot_x, height)
            if not valid:
                raise ValueError(f'Sitting preview is unreachable for {design.key}')
            positions.extend((q1, q2))
        return positions, heights
    if name == 'REARING':
        from rearing_pose import RearingPose
        return RearingPose.target_positions(design.rearing), [0] * 4
    manager = GaitManager(leg, design.gaits)
    if not manager.load_gait(name):
        raise ValueError(f'Cannot generate preview for {name} on {design.key}')
    trajectory = manager.current_trajectories[name]['f']
    # Sample halfway through the first swing segment, away from an endpoint.
    positions = trajectory[max(1, design.gaits[name][6] // 2) % len(trajectory)]
    return positions, [design.gaits[name][2]] * 4


def _detail(name, design):
    if name == 'STANDING':
        return f'Ground {STAND_DEPTH:g} mm below the servo axes  /  feet at the stride centres'
    if name == 'SITTING':
        _, front, rear = design.sitting
        return f'Front {front:g} mm  /  Rear {rear:g} mm'
    if name == 'REARING':
        return f'Rear joint offset {design.rearing[3]:g} deg  /  No waving'
    params = design.gaits[name]
    return f'Rest height {params[2]:g} mm  /  Stride span {params[3]:g} mm'


def make_preview(name, design=None):
    """A static illustration of a representative gait phase or seated posture.

    Limb geometry uses the real solver; chassis spacing and perspective are
    illustrative. This is a preview of commanded positions, not live telemetry.
    """
    if not pygame.font.get_init():
        pygame.font.init()
    design = design or design_for(DEFAULT_DESIGN)
    leg = design.solver()
    positions, heights = _pose_positions(name, design, leg)
    solvers = design.leg_solvers() if name == 'STANDING' else [leg] * 4

    # Render at 2x then downsample for smooth diagonal limbs and text.
    scale = 2
    surface = pygame.Surface((PREVIEW_SIZE[0] * scale, PREVIEW_SIZE[1] * scale))
    surface.fill(eth.PANEL)
    colour = COLORS.get(name, eth.POSE_ACCENT)
    accent = pygame.Color(colour)
    far_accent = eth.mix(colour, eth.PANEL, .45)
    far_shank = eth.mix(eth.SHANK, eth.PANEL, .38)
    def point(p):
        return tuple(round(v * scale) for v in p)
    def text(value, size, color, xy):
        surface.blit(pygame.font.Font(None, size * scale).render(value, True, color), point(xy))
    def line(color, a, b, width=2):
        pygame.draw.line(surface, color, point(a), point(b), width * scale)
    def circle(color, p, radius):
        pygame.draw.circle(surface, color, point(p), radius * scale)
    def poly(color, points):
        pygame.draw.polygon(surface, color, [point(p) for p in points])
    def project(x, side, height):
        if name == 'REARING':
            return 400 + x * 1.65 + side * 1.05, 340 + side * .5 - height * 1.45
        return 400 + x * 2.6 + side * 1.05, 340 + side * .5 - height * 2.75

    # A raised chassis angle illustrates rearing; it is not a predicted balance
    # angle. Actual body tilt depends on load, contacts and joint compliance.
    pitch = math.radians(62)
    if name == 'REARING':
        rear_x, rear_y = leg.fk_solve(*positions[4:6])
        root_height = rear_y * math.cos(pitch) - (rear_x - leg.d / 2) * math.sin(pitch)
    def body_project(x, side, height):
        if name != 'REARING':
            return project(x, side, height)
        relative_x = x + 53
        return project(relative_x * math.cos(pitch) - height * math.sin(pitch) - 40,
                       side, relative_x * math.sin(pitch) + height * math.cos(pitch) + root_height)

    title, description = DESCRIPTIONS.get(name) or EXTRA[name]
    text('STATIC POSE' if name in ('SITTING', 'REARING', 'STANDING') else 'GAIT PREVIEW', 18, accent, (28, 23))
    text(title, 44, eth.TEXT_PRIMARY, (28, 47))
    text(description, 23, eth.TEXT_SECONDARY, (28, 95))
    text(design.title.upper() + ' LEG', 18, eth.TEXT_MUTED, (628, 23))
    # A ground grid supplies context for leg clearance and body tilt.
    for x in range(-100, 101, 25):
        line(eth.GRID, project(x, -58, 0), project(x, 58, 0), 1)
    for side in range(-50, 51, 25):
        line(eth.GRID, project(-105, side, 0), project(105, side, 0), 1)

    # Widths and outlines per link role, so every design reads the same way.
    STYLE = {leg_geometry.CRANK: (6, 10), leg_geometry.PUSHROD: (4, 8),
             leg_geometry.BONE: (6, 10), leg_geometry.FRAME: (2, 0)}

    # FL, FR, BL, BR; far side is drawn first, chassis next, near side last.
    anchors = [(53, 27), (53, -27), (-53, 27), (-53, -27)]
    def draw_leg(index, far=False):
        body_x, side = anchors[index]
        height = heights[index]
        q1, q2 = (float(q) for q in positions[index * 2:index * 2 + 2])
        pivots = leg_geometry.pivots(solvers[index], q1, q2)
        def local(x, y):
            return body_project(body_x + x - leg.d / 2, side, height - y)
        screen = {key: local(x, y) for key, (x, y) in pivots.items()}
        rod = far_accent if far else accent
        shank = far_shank if far else eth.SHANK
        for a, b, role in leg_geometry.links(pivots):
            width, outline = STYLE[role]
            colour = rod if role == leg_geometry.CRANK else (
                eth.CHASSIS_FILL if role == leg_geometry.FRAME else shank)
            if outline:
                line(eth.OUTLINE, screen[a], screen[b], outline)
            line(colour, screen[a], screen[b], width)
        motors = leg_geometry.motor_pivots()
        for key, position in screen.items():
            if key in motors:
                circle(eth.CHASSIS_FILL, position, 9)
                circle(eth.JOINT_CORE, position, 4)
            elif key != leg_geometry.foot_pivot():
                circle(eth.OUTLINE, position, 5)
                circle(rod, position, 3)
        foot = screen[leg_geometry.foot_pivot()]
        circle(eth.OUTLINE, foot, 7)
        circle(rod, foot, 4)
        if not far:
            label_xy = (foot[0] - 23, 387)
            if name == 'REARING':
                label_xy = (foot[0] + 10, foot[1] - 4) if index == 0 else (foot[0] - 23, foot[1] + 17)
            text('FRONT' if index == 0 else 'REAR', 17, eth.TEXT_MUTED, label_xy)
    draw_leg(3, True)
    draw_leg(1, True)
    corners = [body_project(-69, -24, heights[2]), body_project(69, -24, heights[0]),
               body_project(69, 24, heights[0]), body_project(-69, 24, heights[2])]
    poly(eth.CHASSIS_SHADOW, [(x, y + 13) for x, y in corners])
    poly(eth.CHASSIS_FILL, corners)
    pygame.draw.lines(surface, eth.CHASSIS_EDGE, True, [point(p) for p in corners], 2 * scale)
    # Two battery shapes on the chassis provide a recognizable robot silhouette.
    for side in (-11, 10):
        line(eth.BATTERY, body_project(-36, side, heights[2] * .75 + heights[0] * .25 + 3),
             body_project(36, side, heights[2] * .25 + heights[0] * .75 + 3), 10)
    draw_leg(2)
    draw_leg(0)
    line(accent, (686, 176), (725, 176), 2)
    poly(accent, [(725, 176), (716, 171), (716, 181)])
    text('FRONT', 17, accent, (681, 151))
    line(eth.PANEL_RULE, (28, 420), (772, 420), 1)
    text(_detail(name, design), 22, eth.TEXT_SECONDARY, (28, 437))
    text('Kinematic illustration', 18, eth.TEXT_MUTED, (615, 439))
    return pygame.transform.smoothscale(surface, PREVIEW_SIZE)


def generate_previews(directory, design=None):
    """Write one design's ten pictures into ``directory``."""
    design = design or design_for(DEFAULT_DESIGN)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name in DESCRIPTIONS:
        pygame.image.save(make_preview(name, design), str(directory / f'{name.lower()}.png'))


def generate_all(root):
    """Regenerate every available design's pictures under ``root/<design>/``."""
    from leg_designs import selectable
    root = Path(root)
    written = []
    for design in selectable():
        generate_previews(root / design.asset_dir, design)
        written.append(design.key)
    return written


if __name__ == '__main__':
    keys = generate_all(Path(__file__).resolve().parent.parent / 'docs' / 'poses')
    print(f'Generated {len(DESCRIPTIONS)} pictures for: {", ".join(keys)}.')
