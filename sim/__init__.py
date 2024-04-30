import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
pygame.init()

from sim.ev3 import Battery, Buttons, Light, Screen
from sim.speaker import Speaker
from sim.motors import Motor, Control, Number, Stop
import sim.time as time
from sim.objects import Obj

import dat
from json import load, dump
from tkinter.filedialog import asksaveasfile, askopenfile

from typing import Optional, overload, Awaitable
MaybeAwaitable = None | Awaitable[None]

from kthread import KThread, DoNotPrintException

from sim.mathMethods import (
    scale_sur,
    rotate,
    fixangle
)

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
    
    def simulate(self, function, drivebase, path_plotter=False): # Path plotter can be changed mid-sim, this is just the starting value
        r = True
        rawobjs = dat.get_positions()['Objects']
        objs = [
            Obj(pygame.color.THECOLORS[i.strip('_box').lower()], rawobjs[i]) for i in rawobjs
        ]
        if dat.path_exists():
            path = dat.load_path()
        else:
            path = [drivebase.position]
        font = pygame.font.SysFont(None, 16)
        audioicon = pygame.image.load('sim/ims/audio.png')
        field = pygame.Surface(dat.get_positions()['Rects']['Board_size'][1])
        clock = pygame.time.Clock()
        time.reset()
        fr = 60
        time.set_fr(fr)
        t = None
        def start_thread():
            newt = KThread(target=function, name='Simulated EV3 program', daemon=True)
            newt.start()
            return newt
        if not path_plotter:
            t = start_thread()
        while r:
            self.win.fill((255, 255, 255))
            drivebase()
            
            # Field
            field.fill((255, 255, 255))
            ## set some params
            fieldpos = (10, 10)
            fieldsize = (200, 200)
            _, is_hori, diff, smaller, scale = scale_sur(field, fieldsize, True) # Verbose used for mouse pos
            
            ## Get mouse relative to field
            mpos = pygame.mouse.get_pos()
            mpos = [mpos[0] - fieldpos[0], mpos[1] - fieldpos[1]]
            if is_hori:
                mpos[0] -= diff
            else:
                mpos[1] -= diff
            # use_mpos = not any([mpos[0] < 0 or mpos[0] > smaller.get_width(), mpos[1] < 0 or mpos[1] > smaller.get_height()])
            # if use_mpos
            if pygame.mouse.get_pressed()[0] and path_plotter: # Only if you are pressing down the mouse AND have the path plotter enabled will this ever be True
                drivebase.position = mpos
            mpos[0] /= scale
            mpos[1] /= scale
            
            keeponfield = lambda val, spot: min(max(val, 0), field.get_size()[spot])
            mpos = [keeponfield(round(mpos[0], 1), 0), keeponfield(round(mpos[1], 1), 0)]
            
            # Do some event stuff here before continuing with other objects as this has some effects on some of the objects
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    r = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        r = False
                    elif event.key == pygame.K_p:
                        path_plotter = not path_plotter
                        if path_plotter:
                            t.raise_exc(DoNotPrintException)
                            drivebase.stop()
                        else:
                            drivebase.position = path[0]
                            drivebase.rotation = 90
                            t = start_thread()
                    elif path_plotter:
                        if event.key == pygame.K_r:
                            path = [drivebase.position]
                        elif event.key == pygame.K_i:
                            path.append(drivebase.position)
                        elif event.key == pygame.K_d and len(path) > 1:
                            drivebase.position = path.pop()
                        elif event.key == pygame.K_s:
                            dat.save_path(path)
                        elif event.key == pygame.K_a:
                            f = asksaveasfile(mode='w+', defaultextension='.json', filetypes=[('JSON files', '*.json')])
                            dump(path, f)
                            f.close()
                        elif event.key == pygame.K_o:
                            f = askopenfile(mode='r', filetypes=[('JSON files', '*.json')])
                            path = load(f)
                            f.close()
            
            # More field stuff
            ## Robot
            sze = (100, 100)
            robot = pygame.Surface(sze)
            robot.fill((255, 255, 255))
            pygame.draw.rect(robot, 0, (0, 0, *sze), 15)
            pygame.draw.rect(robot, (255, 255, 255), (0, 0, *sze), 1)
            roboPic = pygame.font.Font(None, 100).render('<=', 1, 0)
            robot.blit(roboPic, ((sze[0] - roboPic.get_width())/2, (sze[1] - roboPic.get_height())/2))
            roted = pygame.transform.rotate(robot, -drivebase.rotation-90)
            field.blit(roted, (drivebase.position[0] - (roted.get_width() / 2), drivebase.position[1] - (roted.get_height() / 2)))
            
            ## Objs
            for o in objs:
                o.draw(field)
            
            ## Put the path on the field
            if path_plotter:
                for i in path:
                    pygame.draw.circle(field, (10, 50, 255), i, 8)
            
            ## Re-render the field with these new objects on it
            fieldsur = scale_sur(field, fieldsize)
            
            # Put it all on the screen
            pygame.draw.rect(self.win, (139, 69, 19), (fieldpos[0] - 7, fieldpos[1] - 7, fieldsize[0] + 14, fieldsize[1] + 14), 8, 1)
            self.win.blit(fieldsur, fieldpos)
            
            # if use_mpos:
            self.win.blit(font.render(str(mpos), 1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 10))
            
            self.win.blit(self.generate_face(), (fieldsize[0]+44, fieldpos[1])) # TODO: In path plotter do not list the key presses until start simulating
            
            self.win.blit(font.render(f"Path plotter {'en' if path_plotter else 'dis'}abled, toggle with 'P'", 1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 30))
            if path_plotter:
                paras = [
                    "R: Restart (empty path)",
                    "I: Insert current position to path",
                    "D: Delete last position on path (and go to it)",
                    "S: Save path to what will be used in the program",
                    "A: Save path to a separate file (save as)",
                    "O: Open a path from a file",
                    "",
                    "Current path:",
                    "[" + ", ".join([f"({str(round(i[0], 2))}, {str(round(i[1], 2))})" for i in path]) + "]"
                ]
                for i in range(len(paras)):
                    self.win.blit(font.render(paras[i], 1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 45 + 10 * i))
            
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

    Old Args: (The args aren't used anymore, but # TODO: Make them used)
        left_motor (Motor): The motor that drives the left wheel.
        right_motor (Motor): The motor that drives the right wheel.
        wheel_diameter (int): Diameter of the wheels in millimeters.
        axle_track (int): Distance between the points where both wheels touch the ground in millimeters.
    """

    def __init__(self):
        # These are drive (distance) SPEED and turn (heading) SPEED PID controllers!
        self.distance_control = Control()  # type: Control
        self.distance_control.pid(100) # Tweak the PIDs of the distance control
        self.heading_control = Control()  # type: Control
        # Set some stuff to do with the heading control to make it slightly different to the distance control
        self.heading_control.pid(60)
        self.heading_control.FRICTION = 10 # Much harder for friction to rotate you than to move you
        self.heading_control.current = 90 # Because we start at a 90 degree rotation
        # distance, turn
        self.driving = [False, False]
        self.goals = [None, None]
        self.driven = [0, 0]
        self.speeds = [0, 0]
        
        # Set starting stuff
        self.position = [0, 0]
        self.rotation = 90
        self.do = Stop.HOLD
        
        # How close you need to be to be considered 'at the target'
        # I know they're big numbers, but o well
        self.distance_tolerance = 10 # mm
        self.angle_tolerance = 2 # deg
        # Not needed yet
        self.prevspeeds = None
        # The current limits: straight_speed (mm/s), straight_acceleration (tuple[accel, deaccel], mm/s²), turn_rate (deg/s), turn_acceleration (tuple[accel, deaccel], deg/s²)
        # self.limits is defined when we run self.settings
        self.limits = [None, None, None, None]
        self.settings(100, 10, 90, 10)
    
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
        self._reset()
        self.driving = [True, False]
        self.goals = [distance, None]
        
        self.do = then
        if wait:
            while not self.done():
                pass

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
        self._reset()
        self.driving = [False, True]
        self.goals[1] = fixangle(angle)
        
        self.do = then
        if wait:
            while not self.done():
                pass

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
        raise NotImplementedError('Function "curve" has not been implemented for the sim yet!')
    
    def done(self, check_distance: bool = True, check_angle: bool = True) -> bool:
        """done() -> bool

        Checks if an ongoing command or maneuver is done.

        Arguments:
            check_distance (bool): Whether or not to check the distance to consider 'done', by default True
            check_angle (bool): Whether or not to check the angle to consider 'done', by default True

        Returns:
            ``True`` if the command is done, ``False`` if not.
        """
        return all([
            (not check_distance) or self.goals[0] is None or abs(self.goals[0]) < self.distance_tolerance,
            (not check_angle) or self.goals[1] is None or abs(self.goals[1]) < self.angle_tolerance
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
            self.distance_control.limits(args[0], args[1])
            self.heading_control.limits(args[2], args[3])

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
        self.goals = [None, None]
        self.driving = [drive_speed != 0, turn_rate != 0]
        self.do = DONTSTOPMOVING

    def stop(self):
        """
        Stops the robot by letting the motors spin freely.
        """
        self._reset()
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
        self._reset()
        self.do = Stop.BRAKE
    
    def _reset(self):
        if self.prevspeeds is not None:
            self.settings(*self.prevspeeds)
            self.prevspeeds = None
        self.driving = [False, False]
        self.goals = [None, None]
    
    def __call__(self) -> None:
        # print(self.goals, self.driving, sep='\n', end='\n\n')
        speeds = [None, None]
        if self.driving[0] or self.goals[0] is not None:
            if self.goals[0] is not None:
                goal = self.goals[0]
            else:
                goal = 100
            newgoalpos = rotate(self.position, (self.position[0], self.position[1] - goal), -self.rotation)
            diff = abs(self.position[0] - newgoalpos[0]) + abs(self.position[1] - newgoalpos[1])
            speeds[0] = self.distance_control(diff, self.do)
        else:
            speeds[0] = self.distance_control(None, self.do, False)
        if self.driving[1] or self.goals[1] is not None:
            if self.goals[1] is not None:
                goal = self.goals[1]
            else:
                goal = 100
            speeds[1] = self.heading_control(goal, self.do)
        else:
            speeds[1] = self.heading_control(None, self.do, False)
        
        self.speeds = speeds
        # print(speeds)
        
        self.driven[0] += self.speeds[0]
        self.driven[1] += self.speeds[1]
        
        if self.goals[0] is not None:
            self.goals[0] -= self.speeds[0]
            if abs(self.goals[0]) <= 0:
                self.goals[0] = None
        if self.goals[1] is not None:
            self.goals[1] -= self.speeds[1]
            if abs(self.goals[1]) <= 0:
                self.goals[1] = None
        
        self.position = rotate(self.position, (self.position[0], self.position[1] - self.speeds[0]), -self.rotation)
        self.rotation += self.speeds[1]
