'''
Written by yufeng.wu0902@gmail.com

Trajectory generation module for Q8bot gaits.
This module contains functions for generating and managing gait trajectories.
'''

import math
# Note: No imports from kinematics_solver needed - leg is passed as parameter

def append_pos_list(list_1, list_2, list_3, list_4):
    """
    Append values to overall movement list in specific order.

    Robot layout:
       Front
    list_1  list_2
    list_3  list_4

    Args:
        list_1: Front-left leg trajectory
        list_2: Front-right leg trajectory
        list_3: Back-left leg trajectory
        list_4: Back-right leg trajectory

    Returns:
        Aggregated position list: [q1_1, q2_1, q1_2, q2_2, q1_3, q2_3, q1_4, q2_4]
    """
    append_list = []
    for i in range(len(list_1)):
        append_list.append([list_1[i][0], list_1[i][1],
                           list_2[i][0], list_2[i][1],
                           list_3[i][0], list_3[i][1],
                           list_4[i][0], list_4[i][1]])
    return append_list


def generate_trot_trajectories(leg, gait_params):
    """
    Generate complete set of trajectories for TROT gait.
    Creates pre-calculated trajectories for all movement types with variable stride lengths.

    Args:
        leg: Kinematics solver instance
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Dictionary mapping movement types to trajectory arrays:
        {
            'f': forward (n x 8),
            'b': backward (n x 8),
            'l': left turn (n x 8),
            'r': right turn (n x 8),
            'fl_0.75': forward-left 75% turn (n x 8),
            'fl_0.5': forward-left 50% turn (n x 8),
            'fr_0.75': forward-right 75% turn (n x 8),
            'fr_0.5': forward-right 50% turn (n x 8),
            'bl_0.75': backward-left 75% turn (n x 8),
            'bl_0.5': backward-left 50% turn (n x 8),
            'br_0.75': backward-right 75% turn (n x 8),
            'br_0.5': backward-right 50% turn (n x 8)
        }
        where n = s1_count + s2_count (complete cycle)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params

    # Generate base single-leg trajectories with different stride scales
    move_full_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0
    )
    move_0_75_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=0.75
    )
    move_0_5_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=0.5
    )
    move_full_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-1.0
    )
    move_0_75_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-0.75
    )
    move_0_5_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-0.5
    )

    # Check for failures
    if any(m is None for m in [move_full_forward, move_0_75_forward, move_0_5_forward,
                                 move_full_backward, move_0_75_backward, move_0_5_backward]):
        return None

    # Phase shift for diagonal gait pattern (trot uses 50% offset)
    len_factor = (s1_count + s2_count) / s1_count
    phase_shift = int(s1_count * len_factor / 2)

    # Create phase-shifted versions for diagonal leg coordination
    def phase_shift_trajectory(traj):
        return traj[phase_shift:] + traj[:phase_shift]

    # Phase-shifted trajectories
    p_full = phase_shift_trajectory(move_full_forward)
    p_0_75 = phase_shift_trajectory(move_0_75_forward)
    p_0_5 = phase_shift_trajectory(move_0_5_forward)
    n_full = phase_shift_trajectory(move_full_backward)
    n_0_75 = phase_shift_trajectory(move_0_75_backward)
    n_0_5 = phase_shift_trajectory(move_0_5_backward)

    # Generate complete trajectory set for all 12 movement types
    # Robot leg layout:
    #       Front
    #   FL        FR
    #   BL        BR
    trajectories = {
        # Straight movements
        'f': append_pos_list(move_full_forward, p_full, p_full, move_full_forward),
        'b': append_pos_list(move_full_backward, n_full, n_full, move_full_backward),
        'l': append_pos_list(move_full_backward, p_full, n_full, move_full_forward),
        'r': append_pos_list(move_full_forward, n_full, p_full, move_full_backward),

        # Forward turns (inside leg has reduced stride)
        'fl_0.75': append_pos_list(move_0_75_forward, p_full, p_0_75, move_full_forward),
        'fl_0.5':  append_pos_list(move_0_5_forward, p_full, p_0_5, move_full_forward),
        'fr_0.75': append_pos_list(move_full_forward, p_0_75, p_full, move_0_75_forward),
        'fr_0.5':  append_pos_list(move_full_forward, p_0_5, p_full, move_0_5_forward),

        # Backward turns (inside leg has reduced stride)
        'bl_0.75': append_pos_list(move_0_75_backward, n_full, n_0_75, move_full_backward),
        'bl_0.5':  append_pos_list(move_0_5_backward, n_full, n_0_5, move_full_backward),
        'br_0.75': append_pos_list(move_full_backward, n_0_75, n_full, move_0_75_backward),
        'br_0.5':  append_pos_list(move_full_backward, n_0_5, n_full, move_0_5_backward),
    }

    return trajectories


def generate_walk_trajectories(leg, gait_params):
    """
    Generate complete set of trajectories for WALK gait.
    Walk uses 4-leg phasing with each leg offset by 25%.

    Args:
        leg: Kinematics solver instance
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Dictionary mapping movement types to trajectory arrays:
        {
            'f': forward (n x 8),
            'b': backward (n x 8),
            'l': left turn (n x 8),
            'r': right turn (n x 8)
        }
        where n = s1_count + s2_count (complete cycle)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params

    # Generate base trajectories
    move_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0
    )
    move_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-1.0
    )

    if move_forward is None or move_backward is None:
        return None

    # Phase shift for walk gait (each leg offset by 25%)
    len_factor = (s1_count + s2_count) / s1_count
    phase_shift_25 = int(s1_count * len_factor / 4)

    # Create phase-shifted versions (4 phases for 4 legs)
    p1 = move_forward
    p2 = move_forward[phase_shift_25:] + move_forward[:phase_shift_25]
    p3 = move_forward[phase_shift_25*2:] + move_forward[:phase_shift_25*2]
    p4 = move_forward[phase_shift_25*3:] + move_forward[:phase_shift_25*3]

    n1 = move_backward
    n2 = move_backward[phase_shift_25:] + move_backward[:phase_shift_25]
    n3 = move_backward[phase_shift_25*2:] + move_backward[:phase_shift_25*2]
    n4 = move_backward[phase_shift_25*3:] + move_backward[:phase_shift_25*3]

    # Robot leg layout:
    #       Front
    #   FL        FR
    #   BL        BR
    trajectories = {
        'f': append_pos_list(p1, p2, p3, p4),      # Forward
        'b': append_pos_list(n1, n2, n3, n4),      # Backward
        'l': append_pos_list(n1, p2, n3, p4),      # Left turn
        'r': append_pos_list(p1, n2, p3, n4),      # Right turn
    }

    return trajectories


def generate_bound_trajectories(leg, gait_params):
    """
    Generate complete set of trajectories for BOUND gait.
    Bound has front and back legs moving together in pairs.

    Args:
        leg: Kinematics solver instance
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Dictionary mapping movement types to trajectory arrays:
        {
            'f': forward (n x 8),
            'b': backward (n x 8)
        }
        where n = s1_count + s2_count (complete cycle)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params

    # Generate base trajectories
    move_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0
    )
    move_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-1.0
    )

    if move_forward is None or move_backward is None:
        return None

    # Phase shift for bound (front/back pairs offset)
    split = int((s2_count + s1_count) / 4)
    p2 = move_forward[split:] + move_forward[:split]
    n2 = move_backward[split:] + move_backward[:split]

    # Robot leg layout (pairs move together):
    #       Front
    #   FL        FR  (same phase)
    #   BL        BR  (offset phase)
    trajectories = {
        'f': append_pos_list(move_forward, move_forward, p2, p2),  # Forward
        'b': append_pos_list(move_backward, move_backward, n2, n2), # Backward
    }

    return trajectories


def generate_pronk_trajectories(leg, gait_params):
    """
    Generate complete set of trajectories for PRONK gait.
    Pronk has all legs moving in phase (jumping).

    Args:
        leg: Kinematics solver instance
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Dictionary mapping movement types to trajectory arrays:
        {
            'f': forward (n x 8),
            'b': backward (n x 8)
        }
        where n = s1_count + s2_count (complete cycle)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params

    # Generate base trajectories
    move_forward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0
    )
    move_backward = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=-1.0
    )

    if move_forward is None or move_backward is None:
        return None

    # All legs move together (no phase shift)
    trajectories = {
        'f': append_pos_list(move_forward, move_forward, move_forward, move_forward),  # Forward
        'b': append_pos_list(move_backward, move_backward, move_backward, move_backward), # Backward
    }

    return trajectories


def _generate_base_trajectories(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0):
    """
    Generate base single-leg trajectory with variable stride length.
    This is the core trajectory calculation separated from the stacking logic.

    Args:
        leg: Kinematics solver instance
        x0, y0: Starting position
        xrange, yrange, yrange2: Range parameters
        s1_count: Lift phase step count
        s2_count: Down phase step count
        stride_scale: float, stride length multiplier (0.0 to 1.0)
                     1.0 = full stride, 0.75 = 75% stride, 0.5 = 50% stride

    Returns:
        List of [q1, q2] joint angles or None on failure
    """
    move_trajectory = []
    x_start = x0 - (xrange * stride_scale) / 2
    x_end = x0 + (xrange * stride_scale) / 2
    x_lift_step = (xrange * stride_scale) / s1_count
    x_down_step = (xrange * stride_scale) / s2_count
    x = x_start

    # Check physical limits
    if y0 - yrange < 5:
        return None

    # Generate trajectory points for complete gait cycle
    for i in range(s1_count + s2_count):
        if i < s1_count:
            # Lift phase: sinusoidal lift trajectory
            x += x_lift_step
            freq = math.pi / s1_count
            y = y0 - math.sin((i + 1) * freq) * yrange
        else:
            # Down phase: sinusoidal down trajectory
            x = x - x_down_step
            freq = math.pi / s2_count
            y = y0 + math.sin((i - s1_count + 1) * freq) * yrange2

        # Solve inverse kinematics
        q1, q2, check = leg.ik_solve(x, y, True, 1)

        # Validate IK solution
        if len(str(q1)) > 5 or len(str(q2)) > 5:
            xr_new, yr_new = xrange - 1, yrange - 1
            if xr_new > 0 and yr_new > 0:
                return _generate_base_trajectories(
                    leg, x0, y0, xr_new, yr_new, yrange2, s1_count, s2_count, stride_scale
                )
            return None

        move_trajectory.append([q1, q2])

    return move_trajectory
