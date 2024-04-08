import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
pygame.init()

from sim.ev3 import Battery, Buttons, Light, Screen
from sim.speaker import Speaker
from sim.motors import Motor, Control, Number, Stop
import sim.time as time

from dat import get_positions

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
    angle = math.radians(-angle)
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
        audioicon = pygame.image.load('sim/ims/audio.png')
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
            
            ## Robot
            sze = (100, 100)
            robot = pygame.Surface(sze)
            robot.fill((255, 255, 255))
            pygame.draw.rect(robot, 0, (0, 0, *sze), 8)
            roboPic = pygame.font.Font(None, 100).render('<=', 1, 0)
            robot.blit(roboPic, ((sze[0] - roboPic.get_width())/2, (sze[1] - roboPic.get_height())/2))
            roted = pygame.transform.rotate(robot, -drivebase.rotation-90)
            field.blit(roted, drivebase.position)
            
            # Put it all on the screen
            fieldpos = (10, 10)
            fieldsize = (200, 200)
            self.win.blit(scale_sur(field, fieldsize), fieldpos)
            pygame.draw.rect(self.win, (139, 69, 19), (*fieldpos, *fieldsize), 8, 1)
            
            
            self.win.blit(self.generate_face(), (fieldsize[0]+44, fieldpos[1]))
            
            if self.speaker.busy:
                self.win.blit(audioicon, (fieldpos[0]+fieldsize[0]+10, fieldpos[1]))
            
            
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

DONTSTOPMOVING = 99

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
        self.distance_control = (Control(), Control())  # type: list[Control]
        self.heading_control = Control()  # type: Control
        self.heading_control.pid(0.3)
        self.heading_control.FRICTION = 0.01 # Much harder for friction to rotate you than to move you
        self.heading_control.current = 90 # Because we start at a 90 degree rotation
        # These are drive (distance) POSITION and turn (heading) POSITION PID controllers!
        # I realised a bit too late that I should've probably made those the speed instead of absolute position, but
        # it's too late to change that now, and also the math is going to be annoying to make and it works so that's all I need from it
        # If I have time, maybe I'll change it??
        self.goals = [None, None, None]
        self.do = Stop.HOLD
        # distance, turn
        self.driven = [0, 0]
        self.speeds = [0, 0]
        # How close you need to be to be considered 'at the target'
        # I know they're big numbers, but o well
        self.distance_tolerance = 10 # mm
        self.angle_tolerance = 2 # deg
        self.nostoptolerances = (20, 5) # dist, angle
        # Not needed yet
        self.prevspeeds = None
        self.prevtolerances = None
        # The current limits: straight_speed (mm/s), straight_acceleration (tuple[accel, deaccel], mm/s²), turn_rate (deg/s), turn_acceleration (tuple[accel, deaccel], deg/s²)
        # self.limits is defined when we run self.settings
        self.limits = [None, None, None, None]
        self.settings(50, 10, 10, 1)
    
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
        if distance == 0:
            return
        self._check_prev()
        newgoals = rotate(self.position, (self.position[0], self.position[1] - distance), -self.rotation)
        newgoals = (round(newgoals[0], 5), round(newgoals[1], 5))
        self.goals = [None, None, None]
        if newgoals[0] != self.position[0]:
            self.goals[0] = newgoals[0]
        if newgoals[1] != self.position[1]:
            self.goals[1] = newgoals[1]
        
        if then == Stop.NONE:
            if self.prevtolerances is None:
                self.prevtolerances = (self.distance_tolerance, self.angle_tolerance)
            self.distance_tolerance = self.nostoptolerances[0]
            self.angle_tolerance = self.nostoptolerances[1]
        
        if wait:
            while not self.done():
                pass
        self.do = then

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
        if angle == 0:
            return
        self._check_prev()
        self.goals = [None, None, self.rotation + angle]
        
        if then == Stop.NONE:
            if self.prevtolerances is None:
                self.prevtolerances = (self.distance_tolerance, self.angle_tolerance)
            self.distance_tolerance = self.nostoptolerances[0]
            self.angle_tolerance = self.nostoptolerances[1]
        
        if wait:
            while not self.done():
                pass
        self.do = then

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

    @property
    def position(self) -> tuple[int,int]:
        return (self.distance_control[0].current, self.distance_control[1].current)

    @property
    def rotation(self) -> int:
        return self.heading_control.current
    
    def done(self, check_distance: bool = True, check_angle: bool = True) -> bool:
        """done() -> bool

        Checks if an ongoing command or maneuver is done.

        Arguments:
            check_distance (bool): Whether or not to check the distance to consider 'done', by default True
            check_angle (bool): Whether or not to check the angle to consider 'done', by default True

        Returns:
            ``True`` if the command is done, ``False`` if not.
        """
        nones = [self.goals[0] is None and self.goals[1] is None, self.goals[2] is None]
        if check_distance and check_angle and all(nones):
            return True
        elif check_angle and not check_distance and nones[1]:
            return True
        elif check_distance and not check_angle and nones[0]:
            return True
        return all([
            (not check_distance) or self.goals[0] is None or abs(self.goals[0] - self.distance_control[0].current) < self.distance_tolerance,
            (not check_distance) or self.goals[1] is None or abs(self.goals[1] - self.distance_control[1].current) < self.distance_tolerance,
            (not check_angle) or self.goals[2] is None or abs(self.goals[2] - self.heading_control.current) < self.angle_tolerance
        ])
    
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
            return tuple(self.limits)
        else:
            args = list(args) + [None for _ in range(4-len(args))]
            for i in range(len(args)):
                if args[i] is not None:
                    self.limits[i] = args[i]
            self.distance_control[0].limits(args[0], args[1])
            self.distance_control[1].limits(args[0], args[1])
            self.heading_control.limits(args[2], args[3])
            if self.prevspeeds is not None:
                self.prevspeeds = self.limits.copy()

    def drive(self, drive_speed: int, turn_rate: int):
        """
        Start driving at the specified speed and turnrate. Both values are measured at the center point between the wheels of the robot.

        Args:
            drive_speed (int): Speed of the robot in millimeters/second.
            turn_rate (int): Turn rate of the robot in degrees/second.
        """
        if self.prevspeeds is None:
            self.prevspeeds = self.limits.copy()
        self.settings(drive_speed, None, turn_rate, None)
        self.goals = [None, None, None]
        self.do = DONTSTOPMOVING

    def stop(self):
        """
        Stops the robot by letting the motors spin freely.
        """
        self._check_prev()
        self.goals = [None, None, None]
        self.do = Stop.COAST

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
        self._check_prev()
        self.goals = [None, None, None]
        self.do = Stop.BRAKE
    
    def _check_prev(self):
        if self.prevspeeds is not None:
            self.settings(*self.prevspeeds)
            self.prevspeeds = None
        if self.prevtolerances is not None:
            self.distance_tolerance = self.prevtolerances[0]
            self.angle_tolerance = self.prevtolerances[1]
            self.prevtolerances = None
    
    def __call__(self) -> None:
        prev_pos = self.position
        prev_ang = self.rotation
        
        if self.do == DONTSTOPMOVING:
            newgoal = rotate(prev_pos, (prev_pos[0], prev_pos[1] + self.limits[0]), self.rotation)
            newgoal = (round(newgoal[0], 5), round(newgoal[1], 5))
            if newgoal[0] != self.position[0]:
                self.distance_control[0](newgoal[0])
            else:
                self.distance_control[0](1, False)
            if newgoal[1] != self.position[1]:
                self.distance_control[1](newgoal[1])
            else:
                self.distance_control[1](1, False)
            
            self.heading_control(self.rotation + self.limits[2])
            
            self.speeds = [(abs(prev_pos[0] - self.position[0]) + abs(prev_pos[1] - self.position[1])), abs(prev_ang - self.rotation)]
        
            self.driven[0] += self.speeds[0]
            self.driven[1] += self.speeds[1]
            return
        
        dogoal = None
        if self.do == Stop.COAST or self.do == Stop.NONE:
            dogoal = 1
        elif self.do == Stop.BRAKE:
            dogoal = 0.5
        elif self.do == Stop.HOLD:
            dogoal = 0
        
        if not self.done(check_angle=False):
            if self.goals[0] is not None:
                self.distance_control[0](self.goals[0])
            else:
                self.distance_control[0](dogoal, False)
            if self.goals[1] is not None:
                self.distance_control[1](self.goals[1])
            else:
                self.distance_control[1](dogoal, False)
        else:
            self.distance_control[0](dogoal, False)
            self.distance_control[1](dogoal, False)
        
        if not self.done(check_distance=False):
            self.heading_control(self.goals[2]) # Why would self.goals[2] ever be None if that's what we're checking in self.done()?
        else:
            self.heading_control(dogoal, False)
        
        self.speeds = [(abs(prev_pos[0] - self.position[0]) + abs(prev_pos[1] - self.position[1])), abs(prev_ang - self.rotation)]
        
        self.driven[0] += self.speeds[0]
        self.driven[1] += self.speeds[1]
