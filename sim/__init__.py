from sim.mathMethods import toPolar
from sim.motors import Motor, Control, Stop

try: # DO NOT IMPORT THINGS if running on the robot
    import os
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
    import pygame
    pygame.init()

    from sim.ev3 import Battery, Buttons, Light, Screen
    from sim.speaker import Speaker
    import sim.time as time
    from sim.objects import Obj

    import dat
    from json import load, dump
    from tkinter.filedialog import asksaveasfile, askopenfile

    from kthread import KThread, DoNotPrintException

    from typing import overload

    from sim.mathMethods import (
        scale_sur,
        rotate,
        fixangle,
        CPOL
    )
except:
    overload = lambda *args: None

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
        start_box_rect = dat.get_positions()['Rects']['start_box']
        start_box_obj = Obj((0, 0, 0), [start_box_rect[0], (start_box_rect[1][0], start_box_rect[0][1]), start_box_rect[1], (start_box_rect[0][0], start_box_rect[1][1])])
        if dat.path_exists():
            path = dat.load_path()
        else:
            path = [[drivebase.position, None]]
        drivebase.position = path[0][0]
        prev_position = [drivebase.position, drivebase.position]
        prev_rotation = [drivebase.rotation, drivebase.rotation]
        font = pygame.font.SysFont(None, 16)
        audioicon = pygame.image.load('sim/ims/audio.png')
        field = pygame.Surface(dat.get_positions()['Rects']['Board_size'][1])
        clock = pygame.time.Clock()
        time.reset()
        fr = 60
        time.set_fr(fr)
        t = None
        def start_thread():
            dat.set_override(path)
            newt = KThread(target=function, name='Simulated EV3 program', daemon=True)
            newt.start()
            return newt
        if not path_plotter:
            t = start_thread()
        while r:
            self.win.fill((255, 255, 255))
            if not t.is_alive():
                drivebase.stop()
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
            
            mpos[0] /= scale
            mpos[1] /= scale
            
            keeponfield = lambda val, spot: min(max(val, 0), field.get_size()[spot])
            mpos = [keeponfield(round(mpos[0], 1), 0), keeponfield(round(mpos[1], 1), 0)]
            
            if pygame.mouse.get_pressed()[0] and path_plotter: # Only if you are pressing down the mouse AND have the path plotter enabled will this ever be True
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    drivebase.position = mpos
                else:
                    add_position = lambda p: (drivebase.position[0] + p[0], drivebase.position[1] + p[1])
                    rot = rotate((0, 0), (0, 1000), -drivebase.rotation)
                    high = add_position(rot)
                    low = add_position((-rot[0], -rot[1]))
                    drivebase.position = CPOL(high, low, mpos)
            
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
                            if t.is_alive(): # An error occurs when you attempt to raise an exception in a thread that has already stopped
                                t.raise_exc(DoNotPrintException)
                            drivebase.stop()
                        else:
                            drivebase.position = path[0][0]
                            drivebase.rotation = 90
                            drivebase.reset()
                            t = start_thread()
                        for i in objs: 
                            i.reset()
                    elif path_plotter:
                        if event.key == pygame.K_r:
                            path = [[drivebase.position, 0]]
                            for i in objs: 
                                i.reset()
                        elif event.key == pygame.K_b:
                            for i in objs: 
                                i.reset()
                        elif event.key == pygame.K_i:
                            path.append([drivebase.position, None])
                        elif event.key == pygame.K_j:
                            path.append([drivebase.position, drivebase.rotation])
                        elif event.key == pygame.K_d and len(path) > 1:
                            last = path.pop()
                            drivebase.position = last[0]
                            if last[1] is not None:
                                drivebase.rotation = last[1]
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
                            drivebase.position = path[0][0]
                            for i in objs: 
                                i.reset()
            
            ## More than once-off events
            if path_plotter:
                ms = pygame.key.get_mods()
                speed = (6 if ms & pygame.KMOD_SHIFT else (1 if ms & pygame.KMOD_CTRL else 3))
                keys = pygame.key.get_pressed()
                if keys[pygame.K_LEFT]:
                    drivebase.rotation -= speed
                elif keys[pygame.K_RIGHT]:
                    drivebase.rotation += speed
            
            # More field stuff
            ## Robot
            sze = (175, 155)
            robot = pygame.Surface(sze)
            robot.fill((255, 255, 255))
            pygame.draw.rect(robot, 0, (0, 0, sze[0], sze[1]), 15)
            pygame.draw.rect(robot, (255, 255, 255), (0, 0, sze[0], sze[1]), 1)
            roboPic = pygame.font.Font(None, 100).render('<=', 1, 0)
            robot.blit(roboPic, ((sze[0] - roboPic.get_width())/2, (sze[1] - roboPic.get_height())/2))
            roted = pygame.transform.rotate(robot, -drivebase.rotation-90)
            field.blit(roted, (drivebase.position[0] - (roted.get_width() / 2), drivebase.position[1] - (roted.get_height() / 2)))
            
            pos = [drivebase.position[0] - (sze[0] / 2), drivebase.position[1] - (sze[1] / 2)]
            rec = (pos, (pos[0] + sze[0], pos[1]), (pos[0] + sze[0], pos[1] + sze[1]), (pos[0], pos[1] + sze[1]))
            roted_rect = [rotate(drivebase.position, i, -drivebase.rotation) for i in rec]
            
            def extend(points, dir):
                return [(i[0], i[1] + dir//2) for i in points] + [(i[0], i[1] + dir//2) for i in points[::-1]]
            
            fork_width = 8
            fork_length = 115
            fork_dist_from_edge = 32 + (fork_width / 2)
            fork_one = extend([(pos[0] + fork_dist_from_edge, pos[1] + 20), (pos[0] + fork_dist_from_edge, pos[1] - fork_length)], -fork_width)
            roted_fone = [rotate(drivebase.position, i, -drivebase.rotation) for i in fork_one]
            pygame.draw.line(field, 0, roted_fone[0], roted_fone[1], fork_width)
            fork_two = extend([(pos[0] + sze[0] - fork_dist_from_edge, pos[1] + 20), (pos[0] + sze[0] - fork_dist_from_edge, pos[1] - fork_length)], fork_width)
            roted_ftwo = [rotate(drivebase.position, i, -drivebase.rotation) for i in fork_two]
            pygame.draw.line(field, 0, roted_ftwo[0], roted_ftwo[1], fork_width)
            
            ## Objs
            for o in objs:
                o.update(field, 
                         [(roted_rect, 0), (roted_fone, 1), (roted_ftwo, 1)], 
                         -toPolar(prev_position[1], drivebase.position)[1]+90, 
                         prev_rotation[1] - drivebase.rotation,
                         drivebase.position)
            
            start_box_obj.update(field, [], None, None, None)
            
            ## Put the path on the field
            for i in path:
                pygame.draw.circle(field, (10, 50, 255), i[0], 8)
            
            ## Re-render the field with these new objects on it
            fieldsur = scale_sur(field, fieldsize)
            
            # Put it all on the screen
            pygame.draw.rect(self.win, (139, 69, 19), (fieldpos[0] - 7, fieldpos[1] - 7, fieldsize[0] + 14, fieldsize[1] + 14), 8, 1)
            self.win.blit(fieldsur, fieldpos)
            
            self.win.blit(self.generate_face(), (fieldsize[0]+44, fieldpos[1])) # TODO: In path plotter do not list the key presses until start simulating
            
            round_list = lambda l: [i if not hasattr(i, '__round__') else round(i, 2) for i in l]
            
            self.win.blit(
                font.render(str(mpos) + "  " + str((round(drivebase.position[0], 2), round(drivebase.position[1], 2), round(drivebase.rotation, 2))), 1, 0), 
                (fieldpos[0], fieldpos[1] + fieldsize[1] + 10)
            )

            self.win.blit(
                font.render("Goal: " + str(round_list(drivebase.goals)) + ", driving: " + str(drivebase.driving),
                            1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 20))

            self.win.blit(
                font.render("Accumulated distance and rotation: " + str((round(drivebase.distance(), 2), round(drivebase.angle(), 2))),
                            1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 30))
            
            self.win.blit(font.render("Path plotter " + ('en' if path_plotter else 'dis') + "abled, toggle with 'P'", 1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 50))
            if path_plotter:
                paras = [
                    "Click (and hold): Move robot forwards/back",
                    "Shift+Click (and hold): Move robot anywhere",
                    "Arrow keys (L&R): Rotate robot (Shift: move faster, Ctrl: move slower)",
                    "",
                    "R: Restart (empty path)",
                    "B: Reset all objects to their original positions",
                    "",
                    "I: Insert current position to path",
                    "J: Insert both position and rotation to path",
                    "D: Delete last position on path (and go to it)",
                    "",
                    "S: Save path to what will be used in the program",
                    "A: Save path to a separate file (save as)",
                    "O: Open a path from a file",
                    "",
                    "Current path:",
                    "[" + ", ".join(["(" + str(round(i[0][0], 2)) + ", " + str(round(i[0][1], 2)) + ", " + str(None if i[1] is None else round(i[1], 2)) + ")" for i in path]) + "]"
                ]
                for i in range(len(paras)):
                    self.win.blit(font.render(paras[i], 1, 0), (fieldpos[0], fieldpos[1] + fieldsize[1] + 65 + 10 * i))
            
            if self.speaker.busy:
                self.win.blit(audioicon, (fieldpos[0]+fieldsize[0]+10, fieldpos[1]))
            
            # Update screen
            pygame.display.update()
            clock.tick(fr)
            time.tick()
            if prev_position[0] != drivebase.position:
                prev_position[1] = prev_position[0]
                prev_position[0] = drivebase.position
            if prev_rotation[0] != drivebase.rotation:
                prev_rotation[1] = prev_rotation[0]
                prev_rotation[0] = drivebase.rotation
    
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
        self.settings(1000, 100, 110, 20)
    
    def straight(
        self, distance, then: Stop = Stop.HOLD, wait: bool = True
    ):
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
        self, angle, then: Stop = Stop.HOLD, wait: bool = True
    ):
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
        self, radius, angle, then: Stop = Stop.HOLD, wait: bool = True
    ):
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
        straight_speed = None,
        straight_acceleration = None,
        turn_rate = None,
        turn_acceleration = None,
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
            speeds[0] = self.distance_control(goal, self.do)
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
