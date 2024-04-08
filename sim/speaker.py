from typing import Union, Iterable
import numpy as np
import pyttsx3, pygame, math

from pybricks.media.ev3dev import SoundFile
from sim.converts import ConvertSoundFile

MODEL_NAMES = {'af': 'afrikaans', 'an': 'aragonese', 'bg': 'bulgarian', 'bs': 'bosnian', 'ca': 'catalan', 'cs': 'czech', 'cy': 'welsh', 'da': 'danish', 'de': 'german', 'el': 'greek', 'en': 'default', 'en-gb': 'english', 'en-sc': 'en-scottish', 'en-uk-north': 'english-north', 'en-uk-rp': 'english_rp', 'en-uk-wmids': 'english_wmids', 'en-us': 'english-us', 'en-wi': 'en-westindies', 'eo': 'esperanto', 'es': 'spanish', 'es-la': 'spanish-latin-am', 'et': 'estonian', 'fa': 'Persian+English-UK', 'fa-pin': 'persian-pinglish', 'fi': 'finnish', 'fr-be': 'french-Belgium', 'fr-fr': 'french', 'ga': 'irish-gaeilge', 'grc': 'greek-ancient', 'hi': 'hindi', 'hr': 'croatian', 'hu': 'hungarian', 'hy': 'armenian', 'hy-west': 'armenian-west', 'id': 'indonesian', 'is': 'icelandic', 'it': 'italian', 'jbo': 'lojban', 'ka': 'georgian', 'kn': 'kannada', 'ku': 'kurdish', 'la': 'latin', 'lfn': 'lingua_franca_nova', 'lt': 'Lithuanian', 'lv': 'latvian', 'mk': 'macedonian', 'ml': 'malayalam', 'ms': 'malay', 'ne': 'nepali', 'nl': 'dutch', 'no': 'norwegian', 'pa': 'punjabi', 'pl': 'polish', 'pt-br': 'brazil', 'pt-pt': 'portugal', 'ro': 'romanian', 'ru': 'russian', 'sk': 'slovak', 'sq': 'albanian', 'sr': 'serbian', 'sv': 'swedish', 'sw': 'swahili-test', 'ta': 'tamil', 'tr': 'turkish', 'vi': 'vietnam', 'vi-hue': 'vietnam_hue', 'vi-sgn': 'vietnam_sgn', 'zh': 'Mandarin', 'zh-yue': 'cantonese'}

UNUSED_MODELS = ['bengali', 'basque-test', 'Persian+English-US', 'gujarati-test', 'interlingua', 'telugu-test']

bits = 16
#the number of channels specified here is NOT 
#the channels talked about here http://www.pygame.org/docs/ref/mixer.html#pygame.mixer.get_num_channels

class Speaker:
    """
    A class to represent the speaker member of the EV3Brick class!
    """

    def __init__(self):
        self.engine = pyttsx3.init()
        self.beepvolume = 1
        self.busy = False

    def beep(self, frequency: int = 500, duration: int = 100):
        """
        Play a beep/tone.

        Args:
            frequency (int): Frequency of the beep in Hertz. Frequencies below 100 are treated as 100.
            duration (int): Duration of the beep in milliseconds. If the duration is less than 0, then the method returns immediately, and the frequency play continues to play indefinitely.
        """
        # Thanks https://stackoverflow.com/questions/7816294/simple-pygame-audio-at-a-frequency for the idea and https://stackoverflow.com/questions/48043004/how-do-i-generate-a-sine-wave-using-python
        # Set sone vars
        volume = 1
        forever = duration < 0

        sample_rate = 44100
        n_samples = lambda time: int(round(time*sample_rate))
        duration = n_samples(duration / 1000)
        
        fade_in = n_samples(0.01)
        fade_out = n_samples(0.02)
        
        #kernel = 0.5**np.arange(5)

        samples = np.arange(duration) / sample_rate
        left_signal = np.sin(2 * np.pi * (frequency-50) * samples)
        left_signal *= 32767
        #left_signal[-fade_out:] = np.convolve(left_signal[-fade_out:], kernel, mode='full')[:len(left_signal[-fade_out:])]
        
        right_signal = np.sin(2 * np.pi * (frequency+50) * samples)
        right_signal *= 32767
        
        buf = np.array(tuple(zip(left_signal, right_signal)))
        buf = np.int16(buf)
        if not forever:
            buf[:fade_in] = [buf[i]*(i / fade_in) for i in range(len(buf[:fade_in]))]
            buf[-fade_out:] = [buf[fade_out+i]*((duration-i) / duration) for i in range(len(buf[-fade_out:]))]

            sound = pygame.sndarray.make_sound(buf)
            sound.set_volume(volume)
            chan = sound.play()
            self.busy = True
            while chan.get_busy():
                pass
            self.busy = False
        else:
            sound1 = pygame.sndarray.make_sound(buf)
            sound1.set_volume(volume)
            buf[:fade_in] = [buf[i]*(i / fade_in) for i in range(len(buf[:fade_in]))]
            sound2 = pygame.sndarray.make_sound(buf)
            sound2.set_volume(volume)
            chan = sound2.play()
            self.busy = True
            while chan.get_busy():
                pass
            self.busy = False
            sound1.play(-1)

    def play_notes(self, notes: Iterable[str], tempo: int = 120):
        """
        Plays a sequence of musical notes.

        For example, you can play: ['C4/4', 'C4/4', 'G4/4', 'G4/4'].

        Args:
            notes (Iterable[str]): A sequence of notes to be played (see format below).
            tempo (int): Beats per minute where a quarter note is one beat.

        Note:
            Each note is a string with the following format:
                - The first character is the name of the note, A to G or R for a rest.
                # (sharp) or b (flat). B#/Cb and E#/Fb are not allowed.
                - Note names can also include an accidental
                - The note name is followed by the octave number 2 to 8. For example C4 is middle C. The octave changes to the next number at the note C, for example, B3 is the note below middle C (C4).
                - The octave is followed by / and a number that indicates the size of the note. For example /4 is a quarter note, /8 is an eighth note and so on.
                - This can optionally followed by a . to make a dotted note. Dotted notes are 1-1/2 times as long as notes without a dot.
                - The note can optionally end with a _ which is a tie or a slur. This causes there to be no pause between this note and the next note.
        """
        ... # TODO: This

    def play_file(self, file_name: Union[str, SoundFile]):
        """
        Plays a sound file.

        Args:
            file_name (str or SoundFile): Path to the sound file, including the file extension.
        """
        self.busy = True
        wave = ConvertSoundFile(file_name)
        control = wave.play()
        control.wait_done()
        self.busy = False

    def say(self, text: str):
        """
        Says a given text string.

        You can configure the language and voice of the text using set_speech_options().

        Args:
            text (str): What to say.
        """
        self.busy = True
        self.engine.say(text)
        self.engine.runAndWait()
        self.busy = False

    def set_speech_options(self, language: str = None, voice: str = None, speed: int = None, pitch: int = None):
        """
        Configures speech settings used by the say() method.

        Any options that is set to None will not be changed. If an option is set to an invalid value say() will use the default value instead.
        
        Args:
            language (str): Language of the text. For example, you can choose 'en' (English) or 'de' (German). A list of all available languages is given below.
            voice (str): The voice to use. For example, you can choose 'f1' (female voice variant 1) or 'm3' (male voice variant 3). A list of all available voices is given below.
            speed (int): Number of words per minute.
            pitch (int): Pitch (0 to 99). Higher numbers make the voice higher pitched and lower numbers make the voice lower pitched.
            ## PLEASE NOTE THAT THE PITCH AND VOICE VALUE DOESEN'T DO ANYTHING IN THE SIM DUE TO LACK OF OPTIONS WITH THE TTS LIBRARY USED

        Note:
            You can choose the following languages:
                - 'af': Afrikaans
                - 'an': Aragonese
                - 'bg': Bulgarian
                - 'bs': Bosnian
                - 'ca': Catalan
                - 'cs': Czech
                - 'cy': Welsh
                - 'da': Danish
                - 'de': German
                - 'el': Greek
                - 'en': English (default)
                - 'en-gb': English (United Kingdom)
                - 'en-sc': English (Scotland)
                - 'en-uk-north': English (United Kingdom, Northern)
                - 'en-uk-rp': English (United Kingdom, Received Pronunciation)
                - 'en-uk-wmids': English (United Kingdom, West Midlands)
                - 'en-us': English (United States)
                - 'en-wi': English (West Indies)
                - 'eo': Esperanto
                - 'es': Spanish
                - 'es-la': Spanish (Latin America)
                - 'et': Estonian
                - 'fa': Persian
                - 'fa-pin': Persian
                - 'fi': Finnish
                - 'fr-be': French (Belgium)
                - 'fr-fr': French (France)
                - 'ga': Irish
                - 'grc': Greek
                - 'hi': Hindi
                - 'hr': Croatian
                - 'hu': Hungarian
                - 'hy': Armenian
                - 'hy-west': Armenian (Western)
                - 'id': Indonesian
                - 'is': Icelandic
                - 'it': Italian
                - 'jbo': Lojban
                - 'ka': Georgian
                - 'kn': Kannada
                - 'ku': Kurdish
                - 'la': Latin
                - 'lfn': Lingua Franca Nova
                - 'lt': Lithuanian
                - 'lv': Latvian
                - 'mk': Macedonian
                - 'ml': Malayalam
                - 'ms': Malay
                - 'ne': Nepali
                - 'nl': Dutch
                - 'no': Norwegian
                - 'pa': Punjabi
                - 'pl': Polish
                - 'pt-br': Portuguese (Brazil)
                - 'pt-pt': Portuguese (Portugal)
                - 'ro': Romanian
                - 'ru': Russian
                - 'sk': Slovak
                - 'sq': Albanian
                - 'sr': Serbian
                - 'sv': Swedish
                - 'sw': Swahili
                - 'ta': Tamil
                - 'tr': Turkish
                - 'vi': Vietnamese
                - 'vi-hue': Vietnamese (Hue)
                - 'vi-sgn': Vietnamese (Saigon)
                - 'zh': Mandarin Chinese
                - 'zh-yue': Cantonese Chinese

            You can choose the following voices:
                - 'f1': female variant 1
                - 'f2': female variant 2
                - 'f3': female variant 3
                - 'f4': female variant 4
                - 'f5': female variant 5
                - 'm1': male variant 1
                - 'm2': male variant 2
                - 'm3': male variant 3
                - 'm4': male variant 4
                - 'm5': male variant 5
                - 'm6': male variant 6
                - 'm7': male variant 7
                - 'croak': croak
                - 'whisper': whisper
                - 'whisperf': female whisper
        """
        if language is not None:
            if language not in MODEL_NAMES:
                raise ValueError('Invalid language')
            voices = self.engine.getProperty('voices')
            self.system.setProperty('voice', {i.name: i.id for i in voices}[MODEL_NAMES[language]])
        if speed is not None:
            self.engine.setProperty('rate', speed)

    def set_volume(self, volume: int, which: str = "_all_"):
        """
        Sets the speaker volume.

        Args:
            volume (int): Volume of the speaker as a percentage (0 to 100).
            which (str): Which volume to set. 'Beep' sets the volume for beep() and play_notes(). 'PCM' sets the volume for play_file() and say(). '_all_' sets both at the same time.
        """
        if which not in ['Beep', 'PCM', '_all_']:
            raise ValueError('Invalid which')
        if which in ['PCM', '_all_']:
            self.engine.setProperty('volume',volume/100)
        if which in ['Beep', '_all_']:
            self.beepvolume = volume/100
