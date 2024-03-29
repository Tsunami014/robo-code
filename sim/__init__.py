import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
pygame.init()

from sim.ev3 import Battery, Buttons, Light, Screen
from sim.speaker import Speaker
from sim.motors import Motor, Control

from dat import get_positions

from threading import Thread

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
        while r:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    r = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        r = False
            self.win.fill((255, 255, 255))
            
            # Field
            field.fill((255, 255, 255))
            pygame.draw.rect(field, (0, 0, 0), (*drivebase.position, 100, 100))
            
            # Put it all on the screen
            fieldpos = (10, 10)
            fieldsize = (200, 200)
            self.win.blit(scale_sur(field, fieldsize), fieldpos)
            pygame.draw.rect(self.win, (139, 69, 19), (*fieldpos, *fieldsize), 8, 1)
            
            self.win.blit(self.generate_face(), (fieldsize[0]+20, 10))
            
            # Update screen
            pygame.display.update()
            clock.tick(60)
    
    def generate_face(self):
        whole = pygame.Surface((200, 400))
        space = (200 - 178) / 2
        whole.fill((255, 255, 255))
        whole.blit(self.screen.get_sur(), (space, 0))
        pygame.draw.rect(whole, (0, 0, 0), (space, 0, 178, 128), 8, 1)
        f = pygame.font.SysFont(None, 20)
        whole.blit(f.render(str(self.buttons.pressed()), 1, (0, 0, 0)), (20, 160))
        return whole

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
        self.distance_control = Control()  # type: Control
        self.heading_control = Control()  # type: Control
        self.position = (0, 0)

    def straight(self, distance: int):
        """
        Drives straight for a given distance then stops.

        Args:
            distance (int): Distance to travel in millimeters.
        """
        ...

    def turn(self, angle: int):
        """
        Turns in place by a given angle then stops.

        Args:
            angle (int): Angle of the turn in degrees.
        """
        ...

    def settings(self, straight_speed: int = None, straight_acceleration: int = None, turn_rate: int = None, turn_acceleration: int = None) -> tuple[int, int, int, int]:
        """
        Configures the speed and acceleration used by straight() and turn().

        If you give no arguments, this returns the current values as a tuple.

        You can only change the settings while the robot is stopped. This is either before you begin driving or after you call stop().

        Args:
            straight_speed (int):  Speed of the robot during straight() in millimeters/second.
            straight_acceleration (int): Acceleration and deceleration of the robot at the start and end of straight() in millimeters/second^2.
            turn_rate (int): Turn rate of the robot during turn() in degrees/second.
            turn_acceleration (int): Angular acceleration and deceleration of the robot at the start and end of turn() in degrees/second^2.

        Returns:
            Straight speed (millimeters/second), straight acceleration (millimeters/second^2), turn rate (degrees/second), and turn acceleration (degrees/second^2) (if no arguments are provided), None otherwise.
        """
        if straight_speed is None and straight_acceleration is None and turn_rate is None and turn_acceleration is None:
            return (0, 0, 0, 0)
        else:
            return None

    def drive(self, drive_speed: int, turn_rate: int):
        """
        Start driving at the specified speed and turnrate. Both values are measured at the center point between the wheels of the robot.

        Args:
            drive_speed (int): Speed of the robot in millimeters/second.
            turn_rate (int): Turn rate of the robot in degrees/second.
        """
        ...

    def stop(self):
        """
        Stops the robot by letting the motors spin freely.
        """
        ...

    def distance(self) -> int:
        """
        Get the estimated driven distance.

        Returns:
            Driven distance since last reset in millimeters.
        """
        return 0

    def angle(self) -> int:
        """
        Get the estimated rotation angle of the drive base.

        Returns:
            Accumulated angle since last reset in degrees.
        """
        return 0

    def state(self) -> tuple[int, int, int, int]:
        """
        Gets the state of the robot.

        This returns the current distance(), the drive speed, the angle(), and the turn rate.

        Returns:
            Distance in millimeters, Drive Speed in millimeters/second, Angle in degrees, Rotational Speed in degrees/second
        """
        return (0, 0, 0, 0)

    def reset(self):
        """
        Resets the estimated driven distance and angle to 0.
        """
        ...