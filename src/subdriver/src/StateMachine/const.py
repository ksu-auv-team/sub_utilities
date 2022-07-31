#! /usr/bin/env python2
'''Collection of system constants referenced by the system.

Collecting necessary constants in a single location so that
tweaks can be adjusted more easily.
'''

# Should the run be flipped left-to-right?
# Expected to be changed per-run depending on which part of the pool we're in
# True is left-turning, False is right-turning
FLIP_RUN = False

# Default joystick message values
DEFAULT_MSG_AXES = (-0.01, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
DEFAULT_MSG_BUTTONS = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

'''Enumerations for mapping function to position in axes and buttons tuples.

Usage:
  1) Pushing the front/back control stick forward:
    from constants import AXES
    from StateMachine import Sub

    jmsg = Sub.init_joy_msg()
    jmsg[AXES['frontback']] = 1.0
    Sub.publish_joy(jmsg)

  2) Pressing a button:
    from constants import BUTTONS
    from StateMachine import Sub

    jmsg = Sub.init_joy_msg()
    jmsg[BUTTONS['x']] = 1
    Sub.publish_joy(jmsg)
    time.sleep(DEBOUNCE_DELAY)
    jmsg[BUTTONS['x']] = 0
    Sub.publish_joy(jmsg)


'''

AXES = {'rotate': 0, # -1.0 (rotates right) to 1.0 (rotates left) - I know this one is weird, but this is correct
             'vertical': 1, # -1.0 (sinks) to 1.0 (ascends)
             'lt': 2,
             'strafe': 3, # -1.0 (strafe right) to 1.0 (strafe left) - This one is also weird, but is correct
             'frontback': 4, # -1.0 (backwards) to 1.0 (forwards)
             'rt': 5,
             'dpad_h': 6,
             'dpad_v': 7}

BUTTONS = {'a': 0,
                'b': 1,
                'x': 2,
                'y': 3,
                'lb': 4,
                'rb': 5,
                'back': 6,
                'start': 7,
                'xbox': 8,
                'lstickpress': 9,
                'rstickpress': 10}

CLASSES = {'background': 0,
                'barrel': 1,
                'bighole': 2,
                'bootlegger': 3,
                'bootlegger_small_hole': 4,
                'bottle': 5,
                'gate': 6,
                'gman': 7,
                'gman_badge': 8,
                'gman_star': 9,
                "notepad":10,
		"telephone":11,
		"barrel":12,
		"tommy_gun":13}
                # 'pole':25}


# Joystick Messages function map constants
JOY_MAP = {
    # TODO(travis):Actually map these to configured buttons.
    'LAUNCHER_LEFT': 'y',
    'LAUNCHER_RIGHT': 'x',
}

# Launch Tube geometric offsets
'''Offset(Windage) values to apply just before launching torpedo.

Defines offset in MoA (or just degrees, depending upon launcher accuracy and
offset required) to compensate for launcher mounting position in relation
to sub Line-of-Sight.

Named after the practice in shooting sports where instead
of adjusting the rifle's sights to align point-of-aim to point-of-strike, the
shooter adjusts their point-of-aim to offset the deviation from zero. Not a dig
at Mechanical's ability to get the launcher to shoot straight, or the torpedoes
to fly(?) straight, but a provision for unforseen environmentals
(cross-currents, shipping damage, etc.).

windage_angle - launcher horizontal angle relative to central axis of aiming
                line.
windage_offset - offset left/right relative to target.
elevation_angle - launcher vertical angle relative to central axis of aimimng
                  line.
elevation_offset - offset up/down relative to target.
'''
LAUNCHER_LEFT_OFFSET = {
    'WINDAGE_OFFSET': 0,
    'ELEVATION_OFFSET': 0,
}
LAUNCHER_RIGHT_OFFSET = {
    'WINDAGE_OFFSET': 0,
    'ELEVATION_OFFSET': 0,
}

CAMERA_FORWARD_CENTER = {'X': 1, 'Z': 1}
CAMERA_UNDER_CENTER = {'X': 1, 'Y': 1}

SLEEP_TIME = 0.05 #the amount of time to sleep before looking for another frame
WAIT_TIME = 2
PULSE_TIME = 1