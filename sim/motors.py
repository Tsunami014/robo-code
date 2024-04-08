from pybricks.parameters import Direction, Port

import sim.time as time

from typing import Union, Optional, Tuple, overload
from enum import Enum

Number = Union[int, float]

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value

class Stop(Enum):
    """Action after the motor stops or reaches its target."""

    COAST = 0
    """Let the motor move freely."""

    COAST_SMART = 0 # TODO: This; for now we're just making it exactly the same as Stop.COAST
    """
    Let the motor move freely. For the next relative angle maneuver,
    take the last target angle (instead of the current angle) as the new
    starting point. This reduces cumulative errors. This will apply only if the
    current angle is less than twice the configured position tolerance.
    """

    BRAKE = 1
    """Passively resist small external forces."""

    HOLD = 2
    """Keep controlling the motor to hold it at the commanded angle."""

    NONE = 3
    """
    Do not decelerate when approaching the target position. This can be used
    to concatenate multiple motor or drive base maneuvers without stopping. If
    no further commands are given, the motor will proceed to run indefinitely
    at the given speed.
    """

class Control: # Thanks a lot to https://github.com/m-lundberg/simple-pid for the base PID code!
    # TODO: Make trapezoid shape PID: Speed up at start and slow down when almost at goal
    # But make it an option, and use the Stop.
    # But only do that when I go for speed PIDs instead of position PIDs
    """Class to interact with PID controller and settings."""

    scale: int

    """
    Scaling factor between the controlled integer variable
    and the physical output. For example, for a single
    motor this is the number of encoder pulses per degree of rotation.
    """
    
    def __init__(self) -> None:
        self.current = 0
        
        self.FRICTION = 0.2

        self.Kp, self.Ki, self.Kd = 1, 0, 0
        # PID values
        
        self.ControlLimits = [None, None, None] # TODO: This
        # speed, acceleration, torque
        
        self.TargetTs = [0, 0] # TODO: This
        # speed, position; these tell how close to the speed/position you can be to be considered 'at the target'
        
        self.StallTs = [0, 0] # TODO: This
        # speed, time; these tell how much speed/for how long that speed must be reached to be considered stalled

        self.last_time = time.FRAME

        self.clear()
    
    def clear(self):
        """Clears PID computations and coefficients"""

        self.last_error = 0.0

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_input = None
        self._last_error = None

        self.prev_accel = 0

        # Windup Guard... do I keep???
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def __call__(self, goal: int, running: bool = True) -> None:
        now = time.FRAME
        
        if not running:
            output = self.current * ((self.FRICTION * (goal or 0)) or 1) # So the goal turns into the force of the brake:
            # A goal of 0 means no movement at all - break
            # A goal of 1 means it will apply friction
            # A goal of 0.5 means it will have more friction, and a goal of 2 will have less
            # And any other number follows the rules above DO NOT INPUT -1 I DON'T KNOW WHAT'LL HAPPEN
            self._last_input = self.current
            self.last_time = now
            
            self.current = output
            self.prev_accel = output - self.current
            return output
        
        dt = now - self.last_time if (now - self.last_time) else 1e-16

        # Compute error terms
        error = goal - self.current
        d_input = self.current - (self._last_input if (self._last_input is not None) else self.current)
        d_error = error - (self._last_error if (self._last_error is not None) else error)

        # Compute the proportional term
        if not False: # Whether the proportional term should be calculated on the input directly rather than on the error (which is the traditional way). Using proportional-on-measurement avoids overshoot for some types of systems.
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = self._integral  # Optional clamp to avoid integral windup

        if True: # Whether the differential term should be calculated on the input directly rather than on the error (which is the traditional way).
            self._derivative = -self.Kd * d_input / dt
        else:
            self._derivative = self.Kd * d_error / dt

        # Compute final output
        output = self.current + self._proportional + self._integral + self._derivative
        diff = output - self.current
        diff = diff * (dt / time.FRAMERATE) # Convert from mm/frame to mm/sec
        diff = _clamp(diff, (-self.ControlLimits[0], self.ControlLimits[0])) # Control the max speed by limiting how far it can move per time
        if self.ControlLimits[1] is not None: # Control max acceleration by limiting how much it can change over time
            diff = _clamp(diff, (-(abs(self.prev_accel) + self.ControlLimits[1][1]), (abs(self.prev_accel) + self.ControlLimits[1][0])))
        self.prev_accel = diff
        output = self.current + diff

        # Keep track of state
        self._last_input = self.current
        self._last_error = error
        
        self.current = output

        return output

    @overload
    def limits(
        self,
        speed: Optional[Number] = None,
        acceleration: Optional[Number] = None,
        torque: Optional[Number] = None,
    ) -> None: ...

    @overload
    def limits(self) -> Tuple[int, int, int]: ...

    def limits(self, *args):
        """
        limits(speed, acceleration, torque)
        limits() -> Tuple[int, int, int]

        Configures the maximum speed, acceleration, and torque.

        If no arguments are given, this will return the current values.

        The new ``acceleration`` and ``speed`` limit will become effective
        when you give a new motor command. Ongoing maneuvers are not affected.

        Arguments:
            speed (Number, deg/s or Number, mm/s):
                Maximum speed. All speed commands will be capped to this value.
            acceleration (Number, deg/s² or Number, mm/s²):
                Slope of the speed curve when accelerating or decelerating.
                Use a tuple to set acceleration and deceleration separately.
                If one value is given, it is used for both.
            torque (:ref:`torque`):
                Maximum feedback torque during control.
        """
        if args == ():
            return tuple(self.ControlLimits)
        for i in range(len(args)):
            if args[i] is not None:
                if i == 1 and not isinstance(args[i], tuple):
                    tup = (args[i], args[i])
                    self.ControlLimits[i] = tup
                else:
                    self.ControlLimits[i] = args[i]

    @overload
    def pid(
        self,
        kp: Optional[Number] = None,
        ki: Optional[Number] = None,
        kd: Optional[Number] = None,
        integral_deadzone: Optional[Number] = None,
        integral_rate: Optional[Number] = None,
    ) -> None: ...

    @overload
    def pid(self) -> Tuple[int, int, int, int, int]: ...

    def pid(self, *args):
        """pid(kp, ki, kd, integral_deadzone, integral_rate)
        pid() -> Tuple[int, int, int, int, int]

        Gets or sets the PID values for position and speed control.

        If no arguments are given, this will return the current values.

        Arguments:
            kp (int): Proportional position control
                constant. It is the feedback torque per degree of
                error: µNm/deg.
            ki (int): Integral position control constant. It is the feedback
                torque per accumulated degree of error: µNm/(deg s).
            kd (int): Derivative position (or proportional speed) control
                constant. It is the feedback torque per
                unit of speed: µNm/(deg/s).
            integral_deadzone (Number, deg or Number, mm): Zone around the
                target where the error integral does not accumulate errors.
            integral_rate (Number, deg/s or Number, mm/s): Maximum rate at
                which the error integral is allowed to grow.
        """
        if args == ():
            return self.Kp, self.Ki, self.Kd
        self.Kp = args[0] or self.Kp
        if len(args) > 1:
            self.Ki = args[1] or self.Ki
        if len(args) > 2:
            self.Kd = args[2] or self.Kd

    @overload
    def target_tolerances(
        self, speed: Optional[Number] = None, position: Optional[Number] = None
    ) -> None: ...

    @overload
    def target_tolerances(self) -> Tuple[int, int]: ...

    def target_tolerances(self, *args):
        """target_tolerances(speed, position)
        target_tolerances() -> Tuple[int, int]

        Gets or sets the tolerances that say when a maneuver is done.

        If no arguments are given, this will return the current values.

        Arguments:
            speed (Number, deg/s or Number, mm/s): Allowed deviation
                from zero speed before motion is considered complete.
            position (Number, deg or :ref:`distance`): Allowed
                deviation from the target before motion is considered
                complete.
        """
        if args == ():
            return tuple(self.TargetTs)
        for i in range(len(args)):
            if args[i] is not None:
                self.TargetTs[i] = args[i]

    @overload
    def stall_tolerances(
        self, speed: Optional[Number] = None, time: Optional[Number] = None
    ) -> None: ...

    @overload
    def stall_tolerances(self) -> Tuple[int, int]: ...

    def stall_tolerances(self, *args):
        """stall_tolerances(speed, time)
        stall_tolerances() -> Tuple[int, int]

        Gets or sets stalling tolerances.

        If no arguments are given, this will return the current values.

        Arguments:
            speed (Number, deg/s or Number, mm/s): If the controller
                cannot reach this speed for some ``time`` even with maximum
                actuation, it is stalled.
            time (Number, ms): How long the controller has to be below this
                minimum ``speed`` before we say it is stalled.
        """
        if args == ():
            return tuple(self.StallTs)
        for i in range(len(args)):
            self.StallTs[i] = args[i]

class Motor:
    """
    Generic class to control motors with built-in rotation sensors.

    Attributes:
        control (Control): The motor's PID control. 

    Args:
        port (Port): Port to which the motor is connected.
        positive_direction (Direction): Which direction the motor should turn when you give a positive speed or angle.
        gears (Union[List[int], List[List[int]]]): List of gears linked to the motor. For example, [12, 36] represents a gear train with a 12-tooth and a 36-tooth gear. Use a list of lists for multiple gear trains, such as [[12, 36], [20, 16, 40]].

    Note:
        When you specify a gear train, all motor commands and settings are automatically adjusted to account for the resulting gear ratio. The motor direction remains unchanged by this.
    """

    def __init__(self, port: Port, positive_direction: Direction = Direction.CLOCKWISE, gears: Union[list[int], list[list[int]]] = None):
        if port == Port.S1 or port == Port.S2 or port == port.S3 or port == port.S4:
            raise ValueError("Motors must use Port A, B, C, or D.")
        self.control = Control()  # type: Control
        # self.control is for simulating the motor increase and decrease in speed
        self.angle = 0 # degrees
        self.goal = None

    def speed(self) -> int:
        """
        Gets the speed of the motor.

        Returns:
            Motor speed in degrees/second.
        """
        return self.control.current

    def angle(self) -> int:
        """
        Get the rotation angle of the motor.

        Returns:
            Motor angle in degrees.
        """
        return self.angle

    def reset_angle(self, angle: int):
        """
        Sets the accumulated rotation angle of the motor to a desired value.

        Args:
            angle (int): Value to which the angle should be reset in degrees.
        """
        self.angle = angle

    def stop(self):
        """
        Stops the motor and lets it spin freely.

        The motor gradually stops due to friction.
        """
        self.goal = None

    def brake(self):
        """
        Passively brakes the motor.

        The motor stops due to friction, plus the voltage that is generated while the motor is still moving.
        """
        self.goal = 0

    def hold(self):
        """
        Stops the motor and actively holds it at its current angle.
        """
        # TODO: This
        pass

    def run(self, speed: int):
        """
        Runs the motor at a constant speed.

        The motor accelerates to the given speed and keeps running at this speed until you give a new command.

        Args:
            speed (int): Speed of the motor in degrees/second.
        """
        self.goal = speed

    def run_time(self, speed: int, time: int, then: Stop = Stop.HOLD, wait: bool = True):
        """
        Runs the motor at a constant speed for a given amount of time.

        The motor accelerates to the given speed, keeps running at this speed, and then decelerates. The total maneuver lasts for exactly the given amount of time.

        Args:
            speed (int): Speed of the motor in degrees/second.
            time (int): Duration of the maneuver in milliseconds.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing with the rest of the program.
        """
        ... # TODO: This

    def run_angle(self, speed: int, rotation_angle: int, then: Stop = Stop.HOLD, wait: bool = True):
        """
        Run the motor at a constant speed by a given angle. 

        Args:
            speed (int): Speed of the motor in degrees/second.
            rotation_angle (int): Angle by which the motor should rotate in degrees.
            then (Stop): WWhat to do after coming to a standstill.
            wait (bool): Wait for the motor to reach the target before continuing with the rest of the program.
        """
        ... # TODO: This

    def run_target(self, speed: int, target_angle: int, then: Stop = Stop.COAST, wait: bool = True):
        """
        Runs the motor at a constant speed towards a given target angle.

        The direction of rotation is automatically selected based on the target angle. It does matter if speed is positive or negative.

        Args:
            speed (int): Speed of the motor in degrees/second.
            target_angle (int): Angle that the motor should rotate to in degrees.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the motor to reach the target before continuing with the rest of the program.
        """
        ... # TODO: This

    def run_until_stalled(self, speed: int, then: Stop = Stop.COAST, duty_limit: int = None) -> int:
        """
        Runs the motor at a constant speed until it stalls.

        Args:
            speed (int): Speed of the motor in degrees/second.
            then (Stop): What to do after coming to a standstill.
            duty_limit (int) – Torque limit during this command as a percentage (0 - 100). This is useful to avoid applying the full motor torque to a geared or lever mechanism.

        Returns:	
            Angle at which the motor becomes stalled in degrees.
        """
         # TODO: This
        return 0

    def dc(self, duty: int):
        """
        Rotates the motor at a given duty cycle (also known as “power”).

        This method lets you use a motor just like a simple DC motor.

        Args:
            duty (int): The duty cycle as a percentage (-100 to 100).
        """
        ... # TODO: This

    def track_target(self, target_angle: int):
        """
        Tracks a target angle. This is similar to run_target(), but the usual smooth acceleration is skipped: it will move to the target angle as fast as possible. This method is useful if you want to continuously change the target angle.

        Args:
            target_angle (int): Target angle that the motor should rotate to in degrees.
        """
        ... # TODO: This
    
    def __call__(self) -> None:
        if self.goal is None:
            self.control(0, False)
        else:
            self.control(self.goal)
