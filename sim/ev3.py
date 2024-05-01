from pybricks.parameters import Color
from pybricks.media.ev3dev import Font, Image, ImageFile

try: # DO NOT IMPORT THINGS if running on the robot
    from enum import Enum
    
    from sim.converts import ConvertColor, ConvertFont, ConvertImage, ConvertImageFile

    from pygame.key import get_pressed
    import pygame.locals as pgls
except:
    class Enum:
        pass

class Battery:
    """
    A class to pretend to be the battery of the EV3Brick class.
    """ # TODO: Get numbers > 0 (maybe computer battery? XD)

    def voltage(self) -> int:
        """
        Gets the voltage of the battery.

        Returns:
        Voltage in mV.
        """
        return 0

    def current(self) -> int:
        """
        Gets the current supplied by the battery.

        Returns:
        Battery current in mA.
        """
        return 0

class Button(Enum):
    """
    Buttons on a brick or remote.
    """
    LEFT_DOWN = 0
    DOWN = 1
    RIGHT_DOWN = 2
    LEFT = 3
    CENTER = 4
    RIGHT = 5
    LEFT_UP = 6
    UP = 7
    BEACON = 8 # TODO in Buttons class
    RIGHT_UP = 9

class Buttons:
    """
    A class to represent the buttons member of the EV3Brick class for the sim.
    """

    def pressed(self) -> list[Button]:
        """
        Checks which buttons are currently pressed.

        Returns:
        List of pressed buttons.
        """
        btns = []
        keys = get_pressed()

        if keys[pgls.K_SPACE] or keys[pgls.K_RETURN]:
            btns.append(Button.CENTER)
        
        if keys[pgls.K_a] and keys[pgls.K_w]:
            btns.append(Button.LEFT_UP)
        elif keys[pgls.K_a] and keys[pgls.K_s]:
            btns.append(Button.LEFT_DOWN)
        elif keys[pgls.K_d] and keys[pgls.K_s]:
            btns.append(Button.RIGHT_DOWN)
        elif keys[pgls.K_d] and keys[pgls.K_w]:
            btns.append(Button.RIGHT_UP)
        else:
            if keys[pgls.K_w]:
                btns.append(Button.UP)
            if keys[pgls.K_s]:
                btns.append(Button.DOWN)
            if keys[pgls.K_a]:
                btns.append(Button.LEFT)
            if keys[pgls.K_d]:
                btns.append(Button.RIGHT)
        if keys[pgls.K_UP] and Button.UP not in btns:
            btns.append(Button.UP)
        if keys[pgls.K_DOWN] and Button.DOWN not in btns:
            btns.append(Button.DOWN)
        if keys[pgls.K_LEFT] and Button.LEFT not in btns:
            btns.append(Button.LEFT)
        if keys[pgls.K_RIGHT] and Button.RIGHT not in btns:
            btns.append(Button.RIGHT)
        return btns

class Light:
    """
    A class to represent the light member of the EV3Brick class.
    """
    def __init__(self):
        self.state = (0, 0, 0)

    def on(self, color: Color):
        """
        Turns on the light at the specified color.

        Args:
            color (Color): Color of the light. The light turns off if you choose None or a color that is not available. The brick status light supports Color.RED, Color.ORANGE, and Color.GREEN.
        """
        if color not in [Color.RED, Color.ORANGE, Color.GREEN]:
            raise ValueError('Invalid color')
        self.state = ConvertColor(color)

    def off(self):
        """
        Turns off the light.
        """
        self.state = (0, 0, 0)

from pygame.surface import Surface
import pygame.draw

class Screen:
    """
    A stub class to represent the screen member of the EV3Brick class.

    Attributes:
        height (int): The height of the screen in pixels.
        width (int): The width of the screen in pixels.
    """

    def __init__(self):
        self.width = 178  # type: int
        self.height = 128  # type: int
        self.sur = Surface((self.width, self.height))
        self.font = ConvertFont(Font.DEFAULT)
        self.waiting = []
        self.clear()

    def clear(self):
        """
        Clears the screen. All pixels on the screen will be set to Color.WHITE.
        """
        self.sur.fill((255, 255, 255))

    def draw_text(self, x: int, y: int, text: str, text_color: Color = Color.BLACK, background_color: Color = None):
        """
        Draws text on the screen.

        The most recent font set using set_font() will be used or Font.DEFAULT if no font has been set yet.

        Args:
            x (int): The x-axis value where the left side of the text will start.
            y (int): The y-axis value where the top of the text will start.
            text (str): The text to draw.
            text_color (Color): The color used for drawing the text.
            background_color (Color): The color used to fill the rectangle behind the text or None for transparent background.
        """
        self.sur.blit(self.font.render(text, 1, ConvertColor(text_color), ConvertColor(background_color)), (x, y)) # TODO: This

    def print(self, *args, sep: str = "", end: str = "\n"):
        """
        Prints a line of text on the screen.

        This method works like the builtin print() function, but it writes on the screen instead.

        You can set the font using set_font(). If no font has been set, Font.DEFAULT will be used. The text is always printed used black text with a white background.

        Unlike the builtin print(), the text does not wrap if it is too wide to fit on the screen. It just gets cut off. But if the text would go off of the bottom of the screen, the entire image is scrolled up and the text is printed in the new blank area at the bottom of the screen.

        Args:
            args (object): Zero or more objects to print.
            sep (str): Separator that will be placed between each object that is printed.
            end (str): End of line that will be printed after the last object.
        """
        ... # TODO: This

    def set_font(self, font: Font):
        """
        Sets the font used for writing on the screen.

        The font is used for both draw_text() and print().

        Args:
            font (Font): The font to use.
        """
        self.font = ConvertFont(font)

    def load_image(self, source: str | Image | ImageFile):
        """
        Clears this image, then draws the source image centered in the screen.

        Args:
            source (ImageFile, Image, or str): The source Image. If the argument is a string (or ImageFile), then the source image is loaded from file.
        """
        self.clear()
        if isinstance(source, str):
            self.waiting.append((ConvertImageFile(source), ((self.width-160)//2, (self.height-120)//2), None))
        else:
            pass # TODO: When it's an Image

    def draw_image(self, x: int, y: int, source: str | Image | ImageFile, transparent: Color = None):
        """
        Draws the source image on the screen.

        Args:
            x (int): The x-axis value where the left side of the image will start.
            y (int): The y-axis value where the top of the image will start.
            source (ImageFile, Image, str): The source Image. If the argument is a string (or ImageFile), then the source image is loaded from file.
            transparent (Color): The color of image to treat as transparent or None for no transparency.
        """
        if isinstance(source, str):
            self.waiting.append(ConvertImageFile(source), (x, y), ConvertColor(transparent))
        else:
            pass # TODO: When it's an Image

    def draw_pixel(self, x: int, y: int, color: Color = Color.BLACK):
        """
        Draws a single pixel on the screen.

        Args:
            x (int): The x coordinate of the pixel.
            y (int): The y coordinate of the pixel.
            color (Color): The color of the pixel.
        """
        s = Surface((1, 1))
        s.fill(ConvertColor(color))
        self.sur.blit(s, (x, y))

    def draw_line(self, x1: int, y1: int, x2: int, y2: int, width: int = 1, color: Color = Color.BLACK):
        """
        Draws a line on the screen.

        Args:
            x1 (int): The x coordinate of the starting point of the line.
            y1 (int): The y coordinate of the starting point of the line.
            x2 (int): The x coordinate of the ending point of the line.
            y2 (int): The y coordinate of the ending point of the line.
            width (int): The width of the line in pixels.
            color (Color): The color of the line.
        """
        pygame.draw.line(self.sur, ConvertColor(color), (x1, y1), (x2, y2), width)

    def draw_box(self, x1: int, y1: int, x2: int, y2: int, r: int = 0, fill: bool = False, color: Color = Color.BLACK):
        """
        Draws a box on the screen.

        Args:
            x1 (int): The x coordinate of the left side of the box.
            y1 (int): The y coordinate of the top of the box.
            x2 (int): The x coordinate of the right side of the box.
            y2 (int): The y coordinate of the bottom of the box.
            r (int): The radius of the corners of the box.
            fill (bool): If True, the box will be filled with color, otherwise only the outline of the box will be drawn.
            color (Color): The color of the box.
        """
        pygame.draw.rect(self.sur, ConvertColor(color), (x1, y1, x2-x1, y2-y1), int(not fill), (-1 if r==0 else r))

    def draw_circle(self, x: int, y: int, r: int, fill: bool = False, color: Color = Color.BLACK):
        """
        Draws a circle on the screen.

        Args:
            x (int): The x coordinate of the center of the circle.
            y (int): The y coordinate of the center of the circle.
            r (int): The radius of the circle.
            fill (bool): If True, the circle will be filled with color, otherwise only the circumference will be drawn.
            color (Color): The color of the circle.
        """
        pygame.draw.circle(self.sur, ConvertColor(color), (x, y), r, int(not fill))

    def save(self, filename: str):
        """
        Saves the screen as a .png file.

        Args:
            filename (str): The path to the file to be saved.

        Raises:
            TypeError: filename is not a string
            OSError: There was a problem saving the file.
        """
        pygame.image.save(self.sur, filename)
    
    def get_sur(self):
        s = self.sur.copy()
        for sur, pos, colourkey in self.waiting:
            newsur = sur.get_sur()
            if colourkey is not None:
                newsur.set_colorkey(ConvertColor(colourkey))
            s.blit(newsur, pos)
        return s
