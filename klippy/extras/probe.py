# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins
import math
from . import manual_probe

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
"""

class PrinterProbe:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe = mcu_probe
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset')
        self.probe_calibrate_z = 0.
        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)
        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        # Register z_virtual_endstop pin
        self.printer.lookup_object('pins').register_chip('probe', self)
        # Register homing event handlers
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PROBE', self.cmd_PROBE,
                                    desc=self.cmd_PROBE_help)
        self.gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
                                    desc=self.cmd_QUERY_PROBE_help)
        self.gcode.register_command('PROBE_CALIBRATE', self.cmd_PROBE_CALIBRATE,
                                    desc=self.cmd_PROBE_CALIBRATE_help)
        self.gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
                                    desc=self.cmd_PROBE_ACCURACY_help)
        self.gcode.register_command('Z_OFFSET_APPLY_PROBE',
                                    self.cmd_Z_OFFSET_APPLY_PROBE,
                                    desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)
        self.gcode.register_command('CALIBRATE_A_AXIS_2',
                                    self.cmd_CALIBRATE_A_AXIS_2,
                                    desc=self.cmd_CALIBRATE_A_AXIS_help)
        self.gcode.register_command('CALIBRATE_A_AXIS',
                                    self.cmd_CALIBRATE_A_AXIS,
                                    desc=self.cmd_CALIBRATE_A_AXIS_help)
        self.gcode.register_command('CALIBRATE_Z_AXIS',
                                    self.cmd_CALIBRATE_Z_AXIS,
                                    desc=self.cmd_CALIBRATE_Z_AXIS_help)
        self.gcode.register_command('CALIBRATE_C_AXIS',
                                    self.cmd_CALIBRATE_C_AXIS,
                                    desc=self.cmd_CALIBRATE_C_AXIS_help)

    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)
    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)
    def _handle_home_rails_begin(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_begin()
    def _handle_home_rails_end(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_end()
    def _handle_command_error(self):
        try:
            self.multi_probe_end()
        except:
            logging.exception("Multi-probe end")
    def multi_probe_begin(self):
        self.mcu_probe.multi_probe_begin()
        self.multi_probe_pending = True
    def multi_probe_end(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            self.mcu_probe.multi_probe_end()
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe
    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        return self.lift_speed
    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset
    def _probe(self, speed, axis='z', direction='positive'):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        phoming = self.printer.lookup_object('homing')
        pos = toolhead.get_position()
        logging.info(" *** Current position before modification: %s", pos)

        move_distance = 10  # fixed distance to move during probing

        if axis == 'x':
            if direction == 'positive':
                pos[0] += move_distance
            else:
                pos[0] -= move_distance
        elif axis == 'y':
            if direction == 'positive':
                pos[1] += move_distance
            else:
                pos[1] -= move_distance
        else:  # default is 'z'
            pos[2] = self.z_position

        logging.info(" *** Modified position for axis %s in direction %s: %s", axis, direction, pos)

        try:
            epos = phoming.probing_move(self.mcu_probe, pos, speed, axis)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)

        if axis == 'x':
            self.gcode.respond_info("probe at x=%.3f is z=%.6f" % (epos[0], epos[2]))
        elif axis == 'y':
            self.gcode.respond_info("probe at y=%.3f is z=%.6f" % (epos[1], epos[2]))
        else:
            self.gcode.respond_info("probe at z=%.3f is %.6f" % (epos[2], epos[2]))

        logging.info(" *** epos: %s", epos[:3])
        return epos[:3]
    def _move(self, coord, speed):
        self.printer.lookup_object('toolhead').manual_move(coord, speed)
    def _calc_mean(self, positions):
        count = float(len(positions))
        return [sum([pos[i] for pos in positions]) / count
                for i in range(3)]
    def _calc_median(self, positions):
        z_sorted = sorted(positions, key=(lambda p: p[2]))
        middle = len(positions) // 2
        if (len(positions) & 1) == 1:
            # odd number of samples
            return z_sorted[middle]
        # even number of samples
        return self._calc_mean(z_sorted[middle - 1:middle + 1])
    def _calculate_hole_center(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')

        current_position = toolhead.get_position()
        current_x = current_position[0]
        current_y = current_position[1]

        # Probe Y-axis in positive direction
        y_upper_positive = self._probe(speed=self.speed, axis='y', direction='positive')[1]
        gcmd.respond_info("Probed Y upper positive: {}".format(y_upper_positive))
        self._move([current_x, current_y, None], 3000)

        # Probe Y-axis in negative direction
        y_upper_negative = self._probe(speed=self.speed, axis='y', direction='negative')[1]
        gcmd.respond_info("Probed Y upper negative: {}".format(y_upper_negative))
        self._move([current_x, current_y, None], 3000)

        # Calculate the middle Y position
        y_center = (y_upper_positive + y_upper_negative) / 2.0
        gcmd.respond_info("Calculated Y center: {}".format(y_center))

        # Move to the Y center position
        self._move([current_x, y_center, None], 3000)
        gcmd.respond_info("Moving to Y center position: {}".format(y_center))

        # Probe X-axis in positive direction
        x_upper_positive = self._probe(speed=self.speed, axis='x', direction='positive')[0]
        gcmd.respond_info("Probed X upper positive: {}".format(x_upper_positive))
        self._move([current_x, y_center, None], 3000)

        # Probe X-axis in negative direction
        x_upper_negative = self._probe(speed=self.speed, axis='x', direction='negative')[0]
        gcmd.respond_info("Probed X upper negative: {}".format(x_upper_negative))
        self._move([current_x, y_center, None], 3000)

        # Calculate the middle X position
        x_center = (x_upper_positive + x_upper_negative) / 2.0
        gcmd.respond_info("Calculated X center: {}".format(x_center))

        # Move to the X center position
        self._move([x_center, y_center, None], 3000)
        gcmd.respond_info("Moving to X center position: {}".format(x_center))

        return x_center, y_center

    def cmd_CALIBRATE_A_AXIS_2(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        d = 140 # Distance from the center to the probing points

        # Step 1: Move to the initial position for the first hole
        initial_position_first_hole = [-142, 0, 140]
        self._move(initial_position_first_hole, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_first_hole))

        # Probe Z-axis to find the top of the first calibration hole
        z_probe_result_first_hole = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z distance for first hole: {}".format(z_probe_result_first_hole))

        # Move 30mm down from the probed position
        self._move([None, None, z_probe_result_first_hole + 50], 3000)
        gcmd.respond_info("Moving Z down by 10mm from {}".format(z_probe_result_first_hole))

        position_back_left_leg = [-133, d, None]
        self._move(position_back_left_leg, 3000)
        self._move([None, None, z_probe_result_first_hole + 30, 90], 15)

        position_back_left_probed = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z at the back-left leg: {}".format(position_back_left_probed))

        self._move([None, None, z_probe_result_first_hole + 50], 3000)

        position_front_left_leg = [-134, -d, None]
        self._move(position_front_left_leg, 3000)
        self._move([None, None, z_probe_result_first_hole + 30, -90], 15)

        position_front_left_probed = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z at the front-left leg: {}".format(position_front_left_probed))

        self._move([None, None, z_probe_result_first_hole + 50], 3000)
        self._move([0, 0, None], 3000)
        self._move([None, None, None], 15)

        A = position_back_left_probed
        B = position_front_left_probed

        # Calculate the tilt angle
        tilt_angle = math.degrees(math.atan((A - B) / (2 * d)))
        gcmd.respond_info("Calculated tilt angle: {} degrees".format(tilt_angle))

        # Step 3: Apply the correction
        # Rotate the axis by the negative of the tilt angle
        self._move([None, None, None, -tilt_angle], 15)
        gcmd.respond_info("Rotated axis by {} degrees to correct tilt".format(-tilt_angle))

        # Step 3: Redefine the home position
        current_position = toolhead.get_position()
        new_position = [current_position[0], current_position[1], current_position[2],
                        current_position[3] + tilt_angle, current_position[4], current_position[5]]
        toolhead.set_position(new_position, homing_axes=('a',))
        gcmd.respond_info("Redefined A=0 position to current angle: {}".format(-tilt_angle))

        gcmd.respond_info("Calibration completed")

    def cmd_CALIBRATE_A_AXIS(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')

        hole_depth = -.5

        # Step 1: Move to the initial position for the first hole
        initial_position_first_hole = [-144, 0, 140]
        self._move(initial_position_first_hole, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_first_hole))

        # Probe Z-axis to find the top of the first calibration hole
        z_probe_result_first_hole = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z distance for first hole: {}".format(z_probe_result_first_hole))

        # Move 10mm down from the probed position
        self._move([None, None, z_probe_result_first_hole + 10], 3000)
        gcmd.respond_info("Moving Z down by 10mm from {}".format(z_probe_result_first_hole))

        # Move right in X by 6mm from the current X position
        current_position = toolhead.get_position()
        current_x = current_position[0]
        self._move([current_x + 6, None, None], 3000)
        gcmd.respond_info("Moving X right by 6mm from {}".format(current_x))

        # Move Z in positive direction to place the tip of the nozzle in the first calibration hole
        self._move([None, None, z_probe_result_first_hole - hole_depth], 3000)
        gcmd.respond_info("Moving Z to set the tip inside the first probing hole")

        # Calculate the Y center for the first hole
        x_center_first_hole, y_center_first_hole = self._calculate_hole_center(gcmd)
        gcmd.respond_info("Calculated Y center for first hole: {}".format(y_center_first_hole))

        # Redefine the Y=0 position based on the probed Y center using set_position
        current_position = toolhead.get_position()
        new_position = [current_position[0], current_position[1] - y_center_first_hole, current_position[2],
                        current_position[3], current_position[4], current_position[5]]
        toolhead.set_position(new_position, homing_axes=('y',))
        gcmd.respond_info("Redefined Y=0 position to current Y center: {}".format(y_center_first_hole))

        current_position = toolhead.get_position()
        current_x = current_position[0]
        current_z = current_position[2]

        # Move 10mm down from the probed position
        self._move([None, None, current_z + 10], 3000)
        gcmd.respond_info("Moving Z down by 10mm from {}".format(current_z))

        # Move to the initial position for the second hole
        initial_position_second_hole = [current_x + 50, 0, current_z + 10]
        self._move(initial_position_second_hole, 3000)
        gcmd.respond_info(
            "Moving to initial position for second hole X={}, Y={}, Z={}".format(*initial_position_second_hole))

        # Move Z in positive direction to place the tip of the nozzle in the second calibration hole
        self._move([None, None, current_z - 50 - hole_depth], 3000)
        gcmd.respond_info("Moving Z to set the tip inside the second probing hole")

        # Calculate the Y center for the second hole
        x_center_second_hole, y_center_second_hole = self._calculate_hole_center(gcmd)
        gcmd.respond_info("Calculated Y center for second hole: {}".format(y_center_second_hole))

        self._move([None, None, current_z + 10], 3000)

        # Calculate the Y difference between the first and second hole
        y_difference = y_center_second_hole
        gcmd.respond_info("Y difference between first and second hole: {}".format(y_difference))

        # Calculate the angle to rotate the A axis to make it vertical
        hypotenuse = 50.0  # Distance between the two holes in mm
        angle_to_rotate = math.degrees(math.atan2(y_difference, hypotenuse))
        gcmd.respond_info("Angle to rotate the A axis: {}".format(angle_to_rotate))

        # Rotate the A axis to the calculated angle
        self._move([None, None, None, angle_to_rotate], 3000)
        gcmd.respond_info("Rotating A axis to angle: {}".format(angle_to_rotate))

        initial_position_first_hole = [110, 0, z_probe_result_first_hole + 10]
        self._move(initial_position_first_hole, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_first_hole))

        self._move([None, None, z_probe_result_first_hole - hole_depth], 3000)
        gcmd.respond_info("Moving Z to set the tip inside the first probing hole")

        # Calculate the Y center for the first hole
        x_center_first_hole, y_center_first_hole = self._calculate_hole_center(gcmd)
        gcmd.respond_info("Calculated Y center for first hole: {}".format(y_center_first_hole))

        current_position = toolhead.get_position()
        current_x = current_position[0]

        # Move 10mm down from the probed position
        self._move([None, None, z_probe_result_first_hole + 10], 3000)
        gcmd.respond_info("Moving Z down by 10mm from {}".format(z_probe_result_first_hole))

        # Move to the initial position for the second hole
        initial_position_second_hole = [current_x - 50, 0, z_probe_result_first_hole + 10]
        self._move(initial_position_second_hole, 3000)
        gcmd.respond_info(
            "Moving to initial position for second hole X={}, Y={}, Z={}".format(*initial_position_second_hole))

        # Move Z in positive direction to place the tip of the nozzle in the second calibration hole
        self._move([None, None, z_probe_result_first_hole - 50 - hole_depth], 3000)
        gcmd.respond_info("Moving Z to set the tip inside the second probing hole")

        # Calculate the Y center for the second hole
        x_center_second_hole, y_center_second_hole = self._calculate_hole_center(gcmd)
        gcmd.respond_info("Calculated Y center for second hole: {}".format(y_center_second_hole))

        self._move([None, None, z_probe_result_first_hole + 10], 3000)

        # Calculate the Y difference between the first and second hole
        y_difference_right = y_center_first_hole - y_center_second_hole
        gcmd.respond_info("Y difference between first and second hole on the right side: {}".format(y_difference_right))

        angle_to_rotate = math.degrees(math.atan2(y_difference_right, hypotenuse))
        gcmd.respond_info("Angle to rotate the A axis right: {}".format(angle_to_rotate))

        # Rotate the A axis to the calculated angle
        gcmd.respond_info("Rotating A axis right to angle: {}".format(angle_to_rotate))
        self._adjust_motor('stepper_a', angle_to_rotate)

        gcmd.respond_info("Adjustment for second A axis motor applied")


    cmd_CALIBRATE_A_AXIS_help = "Calibrate the A Axis"

    def cmd_CALIBRATE_Z_AXIS(self, gcmd):

        left_top_position = [-144, 0, 140]
        self._move(left_top_position, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*left_top_position))

        # Probe Z-axis to find the top of the calibration hole on the left
        z_probe_result_left = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z left top: {}".format(z_probe_result_left))

        # Move up by 10mm from the probed position
        self._move([None, None, z_probe_result_left + 10], 3000)

        right_top_position = [120, 0, 140]
        self._move(right_top_position, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*right_top_position))

        # Probe Z-axis to find the top of the calibration hole on the right
        z_probe_result_right = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z right top: {}".format(z_probe_result_right))

        # Move up by 10mm from the probed position
        self._move([None, None, z_probe_result_right + 10], 3000)

        # Calculate the difference
        z_difference = z_probe_result_right - z_probe_result_left
        gcmd.respond_info("Z difference: {}".format(z_difference))

        # Adjust Z motors to level the bed
        # Assuming stepper_z controls the left side and stepper_z1 controls the right side
        if z_difference > 0:
            # Right side is higher, lower the right motor
            self._adjust_motor('stepper_z1', -z_difference)
        elif z_difference < 0:
            # Left side is higher, lower the left motor
            self._adjust_motor('stepper_z', z_difference)

        gcmd.respond_info("Z axis leveled")

    cmd_CALIBRATE_Z_AXIS_help = "Calibrate the Z Axis motors position"

    def cmd_CALIBRATE_C_AXIS(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')

        # Step 1: Move to the initial position for the first hole
        initial_position_first_hole = [-144, 0, 140]
        self._move(initial_position_first_hole, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_first_hole))

        # Probe Z-axis to find the top of the first calibration hole
        z_probe_result_first_hole = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z distance for first hole: {}".format(z_probe_result_first_hole))

        # Move 10mm down from the probed position
        self._move([None, None, z_probe_result_first_hole + 10], 3000)
        gcmd.respond_info("Moving Z down by 10mm from {}".format(z_probe_result_first_hole))

        # Move right in X by 6mm from the current X position
        current_position = toolhead.get_position()
        current_x = current_position[0]
        self._move([current_x + 5, None, None], 3000)
        gcmd.respond_info("Moving X right by 5mm from {}".format(current_x))

        # Move Z in positive direction to place the tip of the nozzle in the first calibration hole
        self._move([None, None, z_probe_result_first_hole], 3000)
        gcmd.respond_info("Moving Z to set the tip inside the first probing hole")

        # Calculate the Y center for the first hole
        x_center_first_hole, y_center_first_hole = self._calculate_hole_center(gcmd)
        gcmd.respond_info("Calculated Y center for first hole: {}".format(y_center_first_hole))

        # Redefine the Y=0 position based on the probed Y center using set_position
        current_position = toolhead.get_position()
        new_position = [current_position[0], current_position[1] - y_center_first_hole, current_position[2],
                        current_position[3], current_position[4], current_position[5]]
        toolhead.set_position(new_position, homing_axes=('y',))

        self._move([None, None, current_position[2] + 20], 3000)
        self._move([0, 0, None], 3000)

        bed_border_probing_z = -1.5

        # Step 1: Move to the initial position for the left X probe
        initial_position_left = [-30, 0, None]
        self._move(initial_position_left, 3000)
        self._move([None, None, bed_border_probing_z], 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_left))

        # Probe X-axis to find the left position
        probe_result_x_left = self._probe(speed=self.speed, axis='x', direction='positive')[0]
        gcmd.respond_info("Probed X left: {}".format(probe_result_x_left))
        self._move([probe_result_x_left - 10, None, 10], 3000)

        # Step 2: Move to the initial position for the right X probe
        initial_position_right = [30, 0, None]
        self._move(initial_position_right, 3000)
        self._move([None, None, bed_border_probing_z], 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*initial_position_right))

        # Probe X-axis to find the right position
        probe_result_x_right = self._probe(speed=self.speed, axis='x', direction='negative')[0]
        gcmd.respond_info("Probed X right: {}".format(probe_result_x_right))
        self._move([probe_result_x_right + 10, None, 10], 3000)

        # Calculate the center X position
        center_x = (probe_result_x_left + probe_result_x_right) / 2.0
        gcmd.respond_info("Calculated center X position: {}".format(center_x))

        # Redefine the X=0 position based on the probed center X position using set_position
        current_position = toolhead.get_position()
        new_position = [current_position[0] - center_x, current_position[1], current_position[2], current_position[3],
                        current_position[4], current_position[5]]
        toolhead.set_position(new_position, homing_axes=('x',))
        gcmd.respond_info("Redefined X=0 position to current X center: {}".format(center_x))

        self._move([0, 45, None], 3000)

        top_left_position = [-12, 40, bed_border_probing_z]
        self._move(top_left_position, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*top_left_position))

        probe_result_y_left = self._probe(speed=self.speed, axis='y', direction='negative')[1]
        self._move(top_left_position, 3000)
        gcmd.respond_info("Probed Y left: {}".format(probe_result_y_left))

        top_right_position = [12, 40, bed_border_probing_z]
        self._move(top_right_position, 3000)
        gcmd.respond_info("Moving to initial position X={}, Y={}, Z={}".format(*top_right_position))

        probe_result_y_right = self._probe(speed=self.speed, axis='y', direction='negative')[1]
        self._move(top_right_position, 3000)
        gcmd.respond_info("Probed Y right: {}".format(probe_result_y_right))

        y_difference = probe_result_y_left - probe_result_y_right
        gcmd.respond_info("Y difference between left and right: {}".format(y_difference))

        distance_between_probed_points = 24
        angle_to_rotate = math.degrees(math.atan2(y_difference, distance_between_probed_points))
        gcmd.respond_info("Angle to rotate the C axis: {}".format(angle_to_rotate))

        self._move([None, None, None, None, angle_to_rotate], 3000)
        gcmd.respond_info("Rotating C axis to angle: {}".format(angle_to_rotate))

        current_position = toolhead.get_position()
        new_position = [current_position[0], current_position[1], current_position[2], current_position[3],
                        current_position[4], current_position[5] - angle_to_rotate]
        toolhead.set_position(new_position, homing_axes=('b',))
        gcmd.respond_info("Redefined C=0 position to current C angle: {}".format(angle_to_rotate))

        self._move([None, None, 10], 3000)
        self._move([0, 0, None], 3000)
        gcmd.respond_info("Moving to center position X=0, Y=0, Z=10")

        z_probe_result_center = self._probe(speed=self.speed, axis='z', direction='positive')[2]
        gcmd.respond_info("Probed Z distance at center: {}".format(z_probe_result_center))

        self._move([None, None, z_probe_result_center + .5], 3000)
        gcmd.respond_info("Moving Z up by 0.5mm from {}".format(z_probe_result_center))

        current_position = toolhead.get_position()
        new_position = [current_position[0], current_position[1], current_position[2] - z_probe_result_center - 0.5,
                        current_position[3], current_position[4],
                        current_position[5]]
        toolhead.set_position(new_position, homing_axes=('z',))
        gcmd.respond_info("Redefined Z=0 position to current Z center: {}".format(z_probe_result_center))

        self._move([None, None, 140], 3000)
        self._move([-144, 0, None], 3000)

        probe_axis_distance = self._probe(speed=self.speed, axis='z', direction='positive')[2]

        gcmd.respond_info("A-axis final distance: {}".format(probe_axis_distance))

        self._move([None, None, 140], 3000)
        self._move([0, 0, None], 3000)
        self._move([None, None, 10], 3000)

    cmd_CALIBRATE_C_AXIS_help = "Calibrate the C Axis motors position"

    def _adjust_motor(self, stepper_name, distance):
        # Adjust the specified Z motor by the given distance
        toolhead = self.printer.lookup_object('toolhead')
        force_move = self.printer.lookup_object('force_move')

        stepper = next((s for s in toolhead.get_kinematics().get_steppers() if s.get_name() == stepper_name), None)

        if stepper is None:
            raise self.printer.config_error("Unknown stepper name: %s" % stepper_name)

        force_move.manual_move(stepper, distance, self.speed)

    def run_probe(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        axis = gcmd.get("AXIS", 'z').lower()
        direction = gcmd.get("DIRECTION", 'positive').lower()
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin()
        probexy = self.printer.lookup_object('toolhead').get_position()[:2]
        retries = 0
        positions = []
        axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed, axis=axis, direction=direction)
            positions.append(pos)
            # Check samples tolerance based on the given axis
            axis_positions = [p[axis_index] for p in positions]
            logging.info(" *** axis_positions: %s", axis_positions)
            if max(axis_positions) - min(axis_positions) > samples_tolerance:
                if retries >= samples_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                self._move(probexy + [pos[axis_index] + sample_retract_dist], lift_speed)
        if must_notify_multi_probe:
            self.multi_probe_end()
        # Calculate and return result
        if samples_result == 'median':
            return self._calc_median(positions), axis_index
        return self._calc_mean(positions), axis_index

    cmd_PROBE_help = "Probe height at current XY position"

    def cmd_PROBE(self, gcmd):
        pos, axis_index = self.run_probe(gcmd)
        axis_label = {0: 'x', 1: 'y', 2: 'z'}[axis_index]
        gcmd.respond_info("Result is %s=%.6f" % (axis_label, pos[axis_index]))
        gcmd.respond_info("RESULT=%.6f" % (pos[axis_index],))
        if axis_index == 2:  # If the probed axis is 'z'
            self.last_z_result = pos[2]

    cmd_QUERY_PROBE_help = "Return the status of the probe"

    def cmd_QUERY_PROBE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],))
    def get_status(self, eventtime):
        return {'name': self.name,
                'last_query': self.last_state,
                'last_z_result': self.last_z_result}
    cmd_PROBE_ACCURACY_help = "Probe height accuracy at current XY position"
    def cmd_PROBE_ACCURACY(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
                          " (samples=%d retract=%.3f"
                          " speed=%.1f lift_speed=%.1f)\n"
                          % (pos[0], pos[1], pos[2],
                             sample_count, sample_retract_dist,
                             speed, lift_speed))
        # Probe bed sample_count times
        self.multi_probe_begin()
        positions = []
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed)
            positions.append(pos)
            # Retract
            liftpos = [None, None, pos[2] + sample_retract_dist]
            self._move(liftpos, lift_speed)
        self.multi_probe_end()
        # Calculate maximum, minimum and average values
        max_value = max([p[2] for p in positions])
        min_value = min([p[2] for p in positions])
        range_value = max_value - min_value
        avg_value = self._calc_mean(positions)[2]
        median = self._calc_median(positions)[2]
        # calculate the standard deviation
        deviation_sum = 0
        for i in range(len(positions)):
            deviation_sum += pow(positions[i][2] - avg_value, 2.)
        sigma = (deviation_sum / len(positions)) ** 0.5
        # Show information
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f" % (
                max_value, min_value, range_value, avg_value, median, sigma))
    def probe_calibrate_finalize(self, kin_pos):
        if kin_pos is None:
            return
        z_offset = self.probe_calibrate_z - kin_pos[2]
        self.gcode.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, z_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'z_offset', "%.3f" % (z_offset,))
    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"
    def cmd_PROBE_CALIBRATE(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Perform initial probe
        lift_speed = self.get_lift_speed(gcmd)
        curpos = self.run_probe(gcmd)
        # Move away from the bed
        self.probe_calibrate_z = curpos[2]
        curpos[2] += 5.
        self._move(curpos, lift_speed)
        # Move the nozzle over the probe point
        curpos[0] += self.x_offset
        curpos[1] += self.y_offset
        self._move(curpos, self.speed)
        # Start manual probe
        manual_probe.ManualProbeHelper(self.printer, gcmd,
                                       self.probe_calibrate_finalize)
    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        offset = self.gcode_move.get_status()['homing_origin'].z
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_calibrate = self.z_offset - offset
            self.gcode.respond_info(
                "%s: z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer."
                % (self.name, new_calibrate))
            configfile.set(self.name, 'z_offset', "%.3f" % (new_calibrate,))
    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"

# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = 'OFF'
    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self.raise_probe()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.raise_probe()
    def get_position_endstop(self):
        return self.position_endstop

# Helper code that can probe a series of points and report the
# position at each point.
class ProbePointsHelper:
    def __init__(self, config, finalize_callback, default_points=None):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config settings
        if default_points is None or config.get('points', None) is not None:
            self.probe_points = config.getlists('points', seps=(',', '\n'),
                                                parser=float, count=2)
        def_move_z = config.getfloat('horizontal_move_z', 5.)
        self.default_horizontal_move_z = def_move_z
        self.speed = config.getfloat('speed', 50., above=0.)
        self.use_offsets = False
        # Internal probing state
        self.lift_speed = self.speed
        self.probe_offsets = (0., 0., 0.)
        self.results = []
    def minimum_points(self,n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name))
    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)
    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets
    def get_lift_speed(self):
        return self.lift_speed
    def _move_next(self):
        toolhead = self.printer.lookup_object('toolhead')
        # Lift toolhead
        speed = self.lift_speed
        if not self.results:
            # Use full speed to first probe position
            speed = self.speed
        toolhead.manual_move([None, None, self.horizontal_move_z], speed)
        # Check if done probing
        if len(self.results) >= len(self.probe_points):
            toolhead.get_last_move_time()
            res = self.finalize_callback(self.probe_offsets, self.results)
            if res != "retry":
                return True
            self.results = []
        # Move to next XY probe point
        nextpos = list(self.probe_points[len(self.results)])
        if self.use_offsets:
            nextpos[0] -= self.probe_offsets[0]
            nextpos[1] -= self.probe_offsets[1]
        toolhead.manual_move(nextpos, self.speed)
        return False
    def start_probe(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Lookup objects
        probe = self.printer.lookup_object('probe', None)
        method = gcmd.get('METHOD', 'automatic').lower()
        self.results = []
        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float('HORIZONTAL_MOVE_Z',
                                                def_move_z)
        if probe is None or method != 'automatic':
            # Manual probe
            self.lift_speed = self.speed
            self.probe_offsets = (0., 0., 0.)
            self._manual_probe_start()
            return
        # Perform automatic probing
        self.lift_speed = probe.get_lift_speed(gcmd)
        self.probe_offsets = probe.get_offsets()
        if self.horizontal_move_z < self.probe_offsets[2]:
            raise gcmd.error("horizontal_move_z can't be less than"
                             " probe's z_offset")
        probe.multi_probe_begin()
        while 1:
            done = self._move_next()
            if done:
                break
            pos = probe.run_probe(gcmd)
            self.results.append(pos)
        probe.multi_probe_end()
    def _manual_probe_start(self):
        done = self._move_next()
        if not done:
            gcmd = self.gcode.create_gcode_command("", "", {})
            manual_probe.ManualProbeHelper(self.printer, gcmd,
                                           self._manual_probe_finalize)
    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.results.append(kin_pos)
        self._manual_probe_start()

def load_config(config):
    return PrinterProbe(config, ProbeEndstopWrapper(config))
