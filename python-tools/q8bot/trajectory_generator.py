'''
Written by yufeng.wu0902@gmail.com

Trajectory generation module for Q8bot gaits.
This module contains functions for generating and managing gait trajectories.
'''

import math
from kinematics_solver import *


def generate_gait(leg, dir, gait_params):
    """
    Generate two sets of single-leg position lists (forward + backward).
    Mix and match to create an aggregated list for 4 legs.

    Args:
        leg: Kinematics solver instance
        dir: Direction string ('f', 'b', 'l', 'r', 'fl', 'fr')
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Tuple of (trajectory_list, y_list)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params
    move_p, move_ps, move_n= [], [], []
    x_list_p, x_list_n, x_p_s, y_list = [], [], [], []
    x_list_p2, y_list2 = [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_ps = x_p * 0.5 + 9.75 *0.5
    x_lift_step = xrange / s1_count
    x_down_step = xrange / s2_count
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return dummy_movement(q1_d, q2_d)

    for i in range(s1_count + s2_count):
        if i < s1_count:
            x_p += x_lift_step
            x_ps += x_lift_step * 0.5
            x_n -= x_lift_step
            freq = math.pi / s1_count
            y = y0 - math.sin((i+1) * freq) * yrange
            x_list_p.append(x_p)
            y_list.append(y)
        else:
            x_p = x_p - x_down_step
            x_ps = x_ps - x_down_step * 0.5
            x_n = x_n + x_down_step
            freq = math.pi / s2_count
            y = y0 + math.sin((i-s1_count+1) * freq) * yrange2
            x_list_p2.append(x_p)
            y_list2.append(y)
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_ps, q2_ps, check3 = leg.ik_solve(x_ps, y, True, 1) # smaller gait
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        x_list_n.append(x_n)
        x_p_s.append(x_ps)

        if len(str(q1_p))>5 or len(str(q2_p))>5:
            print("Invalid: ",x_p, x_n, y, check1, check2)
            xr_new, yr_new, yr_new2 = xrange - 1, yrange - 1, yrange2
            print(xr_new, yr_new)
            if xr_new > 0 and yr_new > 0:
                print("Retry with smaller step size")
                return generate_gait(leg, dir, x0, y0, xr_new, yr_new, yr_new2, gait_params)
            return dummy_movement(q1_d, q2_d)
        move_p.append([q1_p, q2_p])
        move_ps.append([q1_ps, q2_ps])
        move_n.append([q1_n, q2_n])

    if stacktype == 'walk':
        return stack_walk(dir, s1_count, s2_count, move_p, move_n), y_list + y_list2
    elif stacktype == 'trot':
        return stack_trot(dir, s1_count, s2_count, move_p, move_n, move_ps, y_list + y_list2), y_list + y_list2
    elif stacktype == 'bound':
        return stack_bound(dir, s1_count, s2_count, move_p, move_n), y_list + y_list2
    elif stacktype == 'pronk':
        return stack_pronk(dir, s1_count, s2_count, move_p, move_n), y_list + y_list2


def stack_trot(dir, s1_count, s2_count, move_p, move_n, move_ps, y_list):
    """
    Stack trajectory arrays for trot gait pattern.
    Trot gait has diagonal leg pairs moving in phase.

    Args:
        dir: Direction ('f', 'b', 'l', 'r', 'fl', 'fr')
        s1_count: Step 1 count (lift phase)
        s2_count: Step 2 count (down phase)
        move_p: Forward movement trajectory
        move_n: Backward movement trajectory
        move_ps: Forward movement trajectory (scaled)
        y_list: Y-coordinate trajectory list

    Returns:
        Stacked position list for all 4 legs
    """
    len_factor = (s1_count+s2_count)/s1_count
    split = int(s1_count * len_factor/2)
    move_p2 = move_p[split:] + move_p[:split]
    move_ps2 = move_ps[split:] + move_ps[:split]
    move_n2 = move_n[split:] + move_n[:split]
    y_2 = y_list[split:] + y_list[:split]

    if dir == 'f':
        return append_pos_list(move_p, move_p2, move_p2, move_p)
    elif dir == 'r':
        return append_pos_list(move_p, move_n2, move_p2, move_n)
    elif dir == 'l':
        return append_pos_list(move_n, move_p2, move_n2, move_p)
    elif dir == 'fr':
        return append_pos_list(move_p, move_ps2, move_p2, move_ps)
    elif dir == 'fl':
        return append_pos_list(move_ps, move_p2, move_ps2, move_p)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n2, move_n2, move_n)


def stack_walk(dir, s1_count, s2_count, move_p, move_n):
    """
    Stack trajectory arrays for walk gait pattern.
    Walk gait has each leg moving sequentially with 1/4 phase offset.

    Args:
        dir: Direction ('f', 'b', 'l', 'r')
        s1_count: Step 1 count (lift phase)
        s2_count: Step 2 count (down phase)
        move_p: Forward movement trajectory
        move_n: Backward movement trajectory

    Returns:
        Stacked position list for all 4 legs
    """
    len_factor = (s1_count+s2_count)/s1_count
    split = int(s1_count * len_factor/4)
    print(split)
    move_p2 = move_p[split:] + move_p[:split]
    move_p3 = move_p[split*2:] + move_p[:split*2]
    move_p4 = move_p[split*3:] + move_p[:split*3]
    move_n2 = move_n[split:] + move_n[:split]
    move_n3 = move_n[split*2:] + move_n[:split*2]
    move_n4 = move_n[split*3:] + move_n[:split*3]

    if dir == 'f':
        return append_pos_list(move_p, move_p2, move_p3, move_p4)
    elif dir == 'r':
        return append_pos_list(move_p, move_n2, move_p3, move_n4)
    elif dir == 'l':
        return append_pos_list(move_n, move_p2, move_n3, move_p4)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n2, move_n3, move_n4)


def stack_bound(dir, s1_count, s2_count, move_p, move_n):
    """
    Stack trajectory arrays for bound gait pattern.
    Bound gait has front and back leg pairs moving together.

    Args:
        dir: Direction ('f', 'b')
        s1_count: Step 1 count (lift phase)
        s2_count: Step 2 count (down phase)
        move_p: Forward movement trajectory
        move_n: Backward movement trajectory

    Returns:
        Stacked position list for all 4 legs
    """
    split = int((s2_count + s1_count)/4)
    move_p2 = move_p[split:] + move_p[:split]
    move_n2 = move_n[split:] + move_n[:split]

    if dir == 'f':
        return append_pos_list(move_p, move_p, move_p2, move_p2)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n, move_n2, move_n2)


def stack_pronk(dir, s1_count, s2_count, move_p, move_n):
    """
    Stack trajectory arrays for pronk gait pattern.
    Pronk gait has all legs moving in phase (jumping).

    Args:
        dir: Direction ('f', 'b')
        s1_count: Step 1 count (lift phase)
        s2_count: Step 2 count (down phase)
        move_p: Forward movement trajectory
        move_n: Backward movement trajectory

    Returns:
        Stacked position list for all 4 legs
    """
    if dir == 'f':
        return append_pos_list(move_p, move_p, move_p, move_p)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n, move_n, move_n)


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


def dummy_movement(q1 = 90, q2 = 90):
    """
    Movement placeholder for invalid or unimplemented gaits.

    Args:
        q1: Joint 1 angle (degrees)
        q2: Joint 2 angle (degrees)

    Returns:
        Tuple of (dummy_trajectory_list, empty_y_list)
    """
    return[[q1 if i % 2 == 0 else q2 for i in range(8)] for j in range(10)], []


def movement_tick(move_list):
    """
    For each step in a movement, cycle through pre-calculated list.

    Args:
        move_list: List of trajectory points

    Returns:
        Tuple of (current_position, updated_move_list)
    """
    pos = move_list[0]
    new_list = move_list[1:] + [move_list[0]]
    return pos, new_list


def generate_trot_trajectories(leg, gait_params):
    """
    Generate complete set of trajectories for TROT gait.
    Creates pre-calculated trajectories for all movement types.

    Args:
        leg: Kinematics solver instance
        gait_params: [stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]

    Returns:
        Dictionary mapping movement types to trajectory arrays:
        {
            'f': forward trajectory (n x 8),
            'b': backward trajectory (n x 8),
            'l': left trajectory (n x 8),
            'r': right trajectory (n x 8),
            'fl': forward-left trajectory (n x 8),
            'fr': forward-right trajectory (n x 8)
        }
        where n = s1_count + s2_count (complete cycle)
    """
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params

    # Generate base single-leg trajectories (forward, backward, and scaled)
    move_p, move_ps, move_n = _generate_base_trajectories(
        leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count
    )

    if move_p is None:
        print("Failed to generate base trajectories")
        return None

    # Phase shift for diagonal gait pattern (trot uses 50% offset)
    len_factor = (s1_count + s2_count) / s1_count
    split = int(s1_count * len_factor / 2)

    # Create phase-shifted versions for diagonal leg coordination
    move_p2 = move_p[split:] + move_p[:split]
    move_ps2 = move_ps[split:] + move_ps[:split]
    move_n2 = move_n[split:] + move_n[:split]

    # Generate complete trajectory set for all movement types
    trajectories = {
        'f':  append_pos_list(move_p, move_p2, move_p2, move_p),    # Forward
        'b':  append_pos_list(move_n, move_n2, move_n2, move_n),    # Backward
        'l':  append_pos_list(move_n, move_p2, move_n2, move_p),    # Left
        'r':  append_pos_list(move_p, move_n2, move_p2, move_n),    # Right
        'fl': append_pos_list(move_ps, move_p2, move_ps2, move_p),  # Forward-Left
        'fr': append_pos_list(move_p, move_ps2, move_p2, move_ps)   # Forward-Right
    }

    return trajectories


def _generate_base_trajectories(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count):
    """
    Generate base single-leg trajectories (forward, backward, scaled forward).
    This is the core trajectory calculation separated from the stacking logic.

    Args:
        leg: Kinematics solver instance
        x0, y0: Starting position
        xrange, yrange, yrange2: Range parameters
        s1_count: Lift phase step count
        s2_count: Down phase step count

    Returns:
        Tuple of (move_p, move_ps, move_n) or (None, None, None) on failure
        - move_p: Forward movement trajectory
        - move_ps: Forward movement trajectory (50% scaled for turning)
        - move_n: Backward movement trajectory
    """
    move_p, move_ps, move_n = [], [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_ps = x_p * 0.5 + 9.75 * 0.5  # Scaled position (50% between x_p and center)
    x_lift_step = xrange / s1_count
    x_down_step = xrange / s2_count

    # Check physical limits
    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return None, None, None

    # Generate trajectory points for complete gait cycle
    for i in range(s1_count + s2_count):
        if i < s1_count:
            # Lift phase: sinusoidal lift trajectory
            x_p += x_lift_step
            x_ps += x_lift_step * 0.5
            x_n -= x_lift_step
            freq = math.pi / s1_count
            y = y0 - math.sin((i + 1) * freq) * yrange
        else:
            # Down phase: sinusoidal down trajectory
            x_p = x_p - x_down_step
            x_ps = x_ps - x_down_step * 0.5
            x_n = x_n + x_down_step
            freq = math.pi / s2_count
            y = y0 + math.sin((i - s1_count + 1) * freq) * yrange2

        # Solve inverse kinematics for each trajectory
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_ps, q2_ps, check3 = leg.ik_solve(x_ps, y, True, 1)
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)

        # Validate IK solution
        if len(str(q1_p)) > 5 or len(str(q2_p)) > 5:
            print(f"Invalid IK solution: x_p={x_p}, x_n={x_n}, y={y}")
            xr_new, yr_new = xrange - 1, yrange - 1
            if xr_new > 0 and yr_new > 0:
                print("Retrying with smaller step size")
                return _generate_base_trajectories(
                    leg, x0, y0, xr_new, yr_new, yrange2, s1_count, s2_count
                )
            return None, None, None

        move_p.append([q1_p, q2_p])
        move_ps.append([q1_ps, q2_ps])
        move_n.append([q1_n, q2_n])

    return move_p, move_ps, move_n
