"""ETH Zurich corporate design palette and the dark theme derived from it.

The seven corporate design colours below are the official ETH values. Every
other colour the GUI uses is derived from them here by tinting (mixing towards
white, the ETH tint scale), shading (mixing towards black) or blending, so the
interface has a single source of truth and no loose hex literals.

ETH publishes no dark variant of the corporate design, so the dark ground is
built from ETH Black and ETH Grey and the accents are taken from the CD tint
scale at levels that stay legible on it. Solid ETH Blue is reserved for filled
elements carrying white text, which is its role in the corporate design.
"""

# The seven ETH corporate design colours, plus ETH Black.
ETH_BLUE = '#215caf'
ETH_PETROL = '#007894'
ETH_GREEN = '#627313'
ETH_BRONZE = '#8e6713'
ETH_RED = '#b7352d'
ETH_PURPLE = '#a7117a'
ETH_GREY = '#6f6f6f'
ETH_BLACK = '#000000'

WHITE = '#ffffff'


def _rgb(colour):
    colour = colour.lstrip('#')
    return tuple(int(colour[i:i + 2], 16) for i in (0, 2, 4))


def _hex(rgb):
    return '#%02x%02x%02x' % tuple(max(0, min(255, round(c))) for c in rgb)


def mix(colour, other, weight):
    """Blend `weight` of `colour` with `1 - weight` of `other`."""
    return _hex([a * weight + b * (1 - weight) for a, b in zip(_rgb(colour), _rgb(other))])


def tint(colour, percent):
    """An ETH tint: `percent` of the colour over white, on the CD tint scale."""
    return mix(colour, WHITE, percent / 100)


def shade(colour, percent):
    """`percent` of the colour over black, for grounds and drop shadows."""
    return mix(colour, ETH_BLACK, percent / 100)


# --- Surfaces -------------------------------------------------------------
BACKGROUND = shade(ETH_GREY, 15)      # Near-black window ground.
PANEL = shade(ETH_GREY, 33)           # Raised cards.
PANEL_RULE = shade(ETH_GREY, 55)      # Hairlines inside a card.
CHIP = shade(ETH_GREY, 48)            # Key caps and unselected chips.
CHIP_DIM = shade(ETH_GREY, 40)
ACCENT_FILL = ETH_BLUE                # Solid CD blue; always carries white text.
STATUS_BAR = ETH_BLUE

# --- Type -----------------------------------------------------------------
TEXT_PRIMARY = WHITE
TEXT_SECONDARY = tint(ETH_GREY, 25)   # Body copy on dark.
TEXT_MUTED = tint(ETH_GREY, 55)       # Hints and captions.
TEXT_ON_ACCENT = WHITE

# --- Accents --------------------------------------------------------------
ACCENT = tint(ETH_BLUE, 55)           # Section labels and the FRONT marker.
POSE_ACCENT = tint(ETH_BRONZE, 70)    # Static poses: sitting and rearing.

# --- Illustration ---------------------------------------------------------
GRID = shade(ETH_GREY, 45)
OUTLINE = shade(ETH_GREY, 12)         # Dark edge drawn under every limb.
JOINT_CORE = tint(ETH_GREY, 30)
SHANK = tint(ETH_GREY, 35)
CHASSIS_FILL = shade(ETH_GREY, 62)
CHASSIS_EDGE = ETH_GREY
CHASSIS_SHADOW = shade(ETH_GREY, 20)
BATTERY = shade(ETH_GREY, 40)

# --- Logger ---------------------------------------------------------------
LOG_INFO = WHITE
LOG_WARNING = tint(ETH_BRONZE, 75)
LOG_ERROR = tint(ETH_RED, 75)

# Eight gaits step down one ETH Blue tint ramp; the two static poses use ETH
# Bronze so a held pose never reads as a gait. The corporate design asks that
# no more than two CD colours be combined, so gaits are told apart by name and
# illustration rather than by hue.
_BLUE_RAMP = [tint(ETH_BLUE, level) for level in (85, 65, 45, 28)]
GAIT_COLORS = {
    'TROT': _BLUE_RAMP[0], 'TROT_HIGH': _BLUE_RAMP[1],
    'TROT_LOW': _BLUE_RAMP[2], 'TROT_FAST': _BLUE_RAMP[3],
    'WALK': _BLUE_RAMP[0], 'CRAWL': _BLUE_RAMP[1],
    'BOUND': _BLUE_RAMP[2], 'PRONK': _BLUE_RAMP[3],
    'SITTING': tint(ETH_BRONZE, 75), 'REARING': tint(ETH_BRONZE, 50),
}
# Filled chip behind a selected entry; poses keep the bronze family.
CHIP_ACTIVE = {name: (ETH_BRONZE if name in ('SITTING', 'REARING') else ETH_BLUE)
               for name in GAIT_COLORS}


if __name__ == '__main__':
    for group in (('ETH_BLUE', ETH_BLUE), ('ETH_BRONZE', ETH_BRONZE), ('BACKGROUND', BACKGROUND),
                  ('PANEL', PANEL), ('CHIP', CHIP), ('ACCENT', ACCENT), ('POSE_ACCENT', POSE_ACCENT),
                  ('TEXT_SECONDARY', TEXT_SECONDARY), ('TEXT_MUTED', TEXT_MUTED),
                  ('GRID', GRID), ('SHANK', SHANK), ('CHASSIS_FILL', CHASSIS_FILL)):
        print('%-16s %s' % group)
    for name, colour in GAIT_COLORS.items():
        print('%-16s %s' % (name, colour))
