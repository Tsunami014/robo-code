import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
pygame.init()

from sim.ev3 import Battery, Buttons, Light, Screen
from sim.speaker import Speaker
from sim.motors import Motor, Control, Number
import sim.time as time

from dat import get_positions

from pybricks.parameters import Stop
from typing import Optional, overload, Awaitable
MaybeAwaitable = None | Awaitable[None]

from threading import Thread
import math

def scale_sur(sur, size):
    scaled = pygame.Surface(size)
    scaled.fill(0)
    scale = min([size[0] / sur.get_width(), size[1] / sur.get_height()])
    newsur = pygame.transform.scale(sur, (int(sur.get_width() * scale), int(sur.get_height() * scale)))
    if newsur.get_width() < size[0]:
        scaled.blit(newsur, ((size[0] - newsur.get_width()) / 2, 0))
    else:
        scaled.blit(newsur, (0, (size[1] - newsur.get_height()) / 2))
    return scaled

def rotate(origin, point, angle): # Thanks, https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python!
    """
    Rotate a point clockwise by a given angle around a given origin.

    The angle should be given in degrees.
    """
    angle = math.radians(-angle) # Below is assuming counter-clockwise rotation :(
    ox, oy = origin
    px, py = point
    
    cos = math.cos(angle)
    sin = math.sin(angle)
    
    ydiff = (py - oy)
    xdiff = (px - ox)

    qx = ox + cos * xdiff - sin * ydiff
    qy = oy + sin * xdiff + cos * ydiff
    return qx, qy

class EV3BrickSim:
    """
    A class to represent a LEGO® MINDSTORMS® EV3 Brick.

    Attributes:
        battery: Represents the battery on the EV3 Brick.
        buttons: Represents the buttons on the EV3 Brick.
        light: Represents the light on the EV3 Brick.
        screen: Represents the screen on the EV3 Brick.
        speaker: Represents the speaker on the EV3 Brick.
    """

    def __init__(self):
        self.win = pygame.display.set_mode((500, 500)) # TODO: Change these numbers to something that works, not fullscreen!
        self.battery = Battery()  # type: Battery
        self.buttons = Buttons()  # type: Buttons
        self.light = Light()  # type: Light
        self.screen = Screen()  # type: Screen
        self.speaker = Speaker()  # type: Speaker
    
    def simulate(self, function, drivebase):
        r = True
        t = Thread(target=function, name='Simulated EV3 program', daemon=True)
        t.start()
        field = pygame.Surface(get_positions()['Rects']['Board_size'][1])
        clock = pygame.time.Clock()
        time.reset()
        fr = 60
        time.set_fr(fr)
        while r:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    r = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        r = False
            self.win.fill((255, 255, 255))
            drivebase()
            
            # Field
            field.fill((255, 255, 255))
            
            # Robot
            sze = (100, 100)
            robot = pygame.Surface(sze)
            robot.fill((255, 255, 255))
            pygame.draw.rect(robot, 0, (0, 0, *sze), 8)
            roboPic = pygame.font.Font(None, 100).render('=>', 1, 0)
            robot.blit(roboPic, ((sze[0] - roboPic.get_width())/2, (sze[1] - roboPic.get_height())/2))
            roted = pygame.transform.rotate(robot, drivebase.rotation - 90) # Because rendered text is in wrong orientation
            field.blit(roted, drivebase.position)
            
            # Put it all on the screen
            fieldpos = (10, 10)
            fieldsize = (200, 200)
            self.win.blit(scale_sur(field, fieldsize), fieldpos)
            pygame.draw.rect(self.win, (139, 69, 19), (*fieldpos, *fieldsize), 8, 1)
            
            self.win.blit(self.generate_face(), (fieldsize[0]+20, 10))
            
            # Update screen
            pygame.display.update()
            clock.tick(fr)
            time.tick()
    
    def generate_face(self):
        whole = pygame.Surface((200, 400))
        space = (200 - 178) / 2
        whole.fill((255, 255, 255))
        whole.blit(self.screen.get_sur(), (space, 0))
        pygame.draw.rect(whole, (0, 0, 0), (space, 0, 178, 128), 8, 1)
        f = pygame.font.SysFont(None, 20)
        whole.blit(f.render(str(self.buttons.pressed()), 1, (0, 0, 0)), (20, 160))
        return whole

DEFAULT_SPEED = 20 # Speed to set to if max speed not set

class DriveBaseSim:
    """
    A robotic vehicle with two powered wheels and an optional support wheel or caster.

    By specifying the dimensions of your robot, this class makes it easy to drive a given distance in millimeters or turn by a given number of degrees.

    Positive distances and drive speeds mean driving forward. Negative means backward.

    Positive angles and turn rates mean turning right. Negative means left. So when viewed from the top, positive means clockwise and negative means counterclockwise.

    Attributes:
        distance_control (Control): The traveled distance and drive speed are controlled by a PID controller. You can use this attribute to change its settings.
        heading_control (Control): The robot turn angle and turn rate are controlled by a PID controller. You can use this attribute to change its settings.

    Args:
        left_motor (Motor): The motor that drives the left wheel.
        right_motor (Motor): The motor that drives the right wheel.
        wheel_diameter (int): Diameter of the wheels in millimeters.
        axle_track (int): Distance between the points where both wheels touch the ground in millimeters.
    """

    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter: int, axle_track: int):
        # TODO: Decide whether to have PIDs for the speed OR the absolute position, and how I'm going to accomplish all the functions with whatever I choose
        self.distance_control = Control()  # type: Control
        self.heading_control = Control()  # type: Control
        self.heading_control.FRICTION = 0.01
        # These are drive (distance) SPEED and turn (heading) SPEED PID controllers!
        self.goals = [None, None]
        self.position = (0, 0)
        self.rotation = 0
        self.dowhendone = Stop.HOLD
        # distance, turn
        self.driven = [0, 0]
        self.speeds = [0, 0]
        self.to = [None, None]
    
    def straight(
        self, distance: Number, then: Stop = Stop.HOLD, wait: bool = True
    ) -> MaybeAwaitable:
        """straight(distance, then=Stop.HOLD, wait=True)

        Drives straight for a given distance and then stops.

        Arguments:
            distance (Number, mm): Distance to travel
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing
                         with the rest of the program.
        """
        self.to[0] = distance
        self.goals[0] = self.distance_control.ControlLimits[0] or DEFAULT_SPEED
        if wait:
            while not self.done():
                pass
        self.dowhendone = then

    def turn(
        self, angle: Number, then: Stop = Stop.HOLD, wait: bool = True
    ) -> MaybeAwaitable:
        """turn(angle, then=Stop.HOLD, wait=True)

        Turns in place by a given angle and then stops.

        Arguments:
            angle (Number, deg): Angle of the turn.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing
                         with the rest of the program.
        """
        self.to[1] = angle
        self.goals[1] = self.heading_control.ControlLimits[0] or DEFAULT_SPEED
        if wait:
            while not self.done():
                pass
        self.dowhendone = then

    def curve(
        self, radius: Number, angle: Number, then: Stop = Stop.HOLD, wait: bool = True
    ) -> MaybeAwaitable:
        """curve(radius, angle, then=Stop.HOLD, wait=True)

        Drives an arc along a circle of a given radius, by a given angle.

        Arguments:
            radius (Number, mm): Radius of the circle.
            angle (Number, deg): Angle along the circle.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing
                         with the rest of the program.
        """
        pass # TODO: This
    
    def done(self) -> bool:
        """done() -> bool

        Checks if an ongoing command or maneuver is done.

        Returns:
            ``True`` if the command is done, ``False`` if not.
        """
        return self.to == [None, None] # Maybe change???
    
    def stalled(self) -> bool:
        """stalled() -> bool

        Checks if the drive base is currently stalled.

        It is stalled when it cannot reach the target speed or position, even
        with the maximum actuation signal.

        Returns:
            ``True`` if the drivebase is stalled, ``False`` if not.
        """
        return False

    def use_gyro(self, use_gyro: bool) -> None:
        """use_gyro(use_gyro)

        Choose ``True`` to use the gyro sensor for turning and driving
        straight. Choose ``False`` to rely only on the motor's built-in
        rotation sensors.

        Arguments:
            use_gyro (bool): ``True`` to enable, ``False`` to disable.
        """
        # TODO: This
        pass

    @overload
    def settings(
        self,
        straight_speed: Optional[Number] = None,
        straight_acceleration: Optional[Number] = None,
        turn_rate: Optional[Number] = None,
        turn_acceleration: Optional[Number] = None,
    ) -> None: ...

    @overload
    def settings(self) -> tuple[int, int, int, int]: ...

    def settings(self, *args):
        """
        settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
        settings() -> Tuple[int, int, int, int]

        Configures the drive base speed and acceleration.

        If you give no arguments, this returns the current values as a tuple.

        The initial values are automatically configured based on your wheel
        diameter and axle track. They are selected such that your robot
        drives at about 40% of its maximum speed.

        The speed values given here do not apply to the :meth:`.drive` method,
        since you provide your own speed values as arguments in that method.

        Arguments:
            straight_speed (Number, mm/s): Straight-line speed of the robot.
            straight_acceleration (Number, mm/s²): Straight-line
                acceleration and deceleration of the robot. Provide a tuple with
                two values to set acceleration and deceleration separately.
            turn_rate (Number, deg/s): Turn rate of the robot.
            turn_acceleration (Number, deg/s²): Angular acceleration and
                deceleration of the robot. Provide a tuple with
                two values to set acceleration and deceleration separately.
        """
        if args == ():
            return (0, 0, 0, 0)
        else:
            self.distance_control.limits(args[0], (None if len(args) < 2 else args[1]))
            if len(args) > 2:
                self.heading_control.limits(args[2], (None if len(args) < 4 else args[3]))

    def drive(self, drive_speed: int, turn_rate: int):
        """
        Start driving at the specified speed and turnrate. Both values are measured at the center point between the wheels of the robot.

        Args:
            drive_speed (int): Speed of the robot in millimeters/second.
            turn_rate (int): Turn rate of the robot in degrees/second.
        """
        self.goals = [drive_speed, turn_rate]

    def stop(self):
        """
        Stops the robot by letting the motors spin freely.
        """
        self.goals = [None, None]

    def distance(self) -> int:
        """
        Get the estimated driven distance.

        Returns:
            Driven distance since last reset in millimeters.
        """
        return self.driven[0]

    def angle(self) -> int:
        """
        Get the estimated rotation angle of the drive base.

        Returns:
            Accumulated angle since last reset in degrees.
        """
        return self.driven[1]

    def state(self) -> tuple[int, int, int, int]:
        """
        Gets the state of the robot.

        This returns the current distance(), the drive speed, the angle(), and the turn rate.

        Returns:
            Distance in millimeters, Drive Speed in millimeters/second, Angle in degrees, Rotational Speed in degrees/second
        """
        return (self.distance(), self.speeds[0], self.angle(), self.speeds[1])

    def reset(self):
        """
        Resets the estimated driven distance and angle to 0.
        """
        self.driven = [0, 0]
    
    def brake(self):
        """
        Passively brakes the motor.

        The motor stops due to friction, plus the voltage that is generated while the motor is still moving.
        """
        self.goal = 0
    
    def __call__(self) -> None:
        if self.goals[0] is None:
            dist_speed = self.distance_control(0, False)
        else:
            dist_speed = self.distance_control(self.goals[0])
        
        if self.goals[1] is None:
            ang_speed = self.heading_control(0, False)
        else:
            ang_speed = self.heading_control(self.goals[1])
        
        self.speeds = [dist_speed, ang_speed]
        
        self.rotation += ang_speed
        
        add_pos = rotate((0, 0), (0, dist_speed), self.rotation)
        self.position = (self.position[0] + add_pos[0], self.position[1] + add_pos[1])
        
        self.driven[0] += dist_speed
        self.driven[1] += ang_speed

        if self.to != [None, None]:
            if self.to[0] is not None:
                self.to[0] -= dist_speed
                if self.to[0] <= 0:
                    self.goals[0] = None
                    self.to[0] = None
            
            if self.to[1] is not None:
                self.to[1] -= ang_speed
                if self.to[1] <= 0:
                    self.goals[1] = None
                    self.to[1] = None
            if self.to == [None, None]: # It *now* finished everything!
                self.dowhendone = Stop.HOLD
