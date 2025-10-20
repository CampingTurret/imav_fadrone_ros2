import RPi.GPIO as GPIO
from time import sleep

SIGNAL_PIN = 11
RETRACT_DC = 5
EXTEND_DC = 10
FREQ_HZ = 50
DEFAULT_PULSE_S = 0.5

class ServoDriver:
    def __init__(self, 
                 signal_pin: int = SIGNAL_PIN,
                 retract_dc: float = RETRACT_DC,
                 extend_dc: float = EXTEND_DC,
                 freq_hz: int = FREQ_HZ,
                 pulse_s: float = DEFAULT_PULSE_S,
                 gpio_mode=GPIO.BOARD):
        
        self.pin = signal_pin
        self.retract_dc = float(retract_dc)
        self.extend_dc = float(extend_dc)
        self.freq_hz = int(freq_hz)
        self.pulse_s = float(pulse_s)
        
        GPIO.setmode(gpio_mode)
        GPIO.setup(self.pin, GPIO.OUT)

        self._pwm = GPIO.PWM(self.pin, self.freq_hz)
        self._pwm.start(self.retract_dc)

    def retract(self):
        self._pwm.ChangeDutyCycle(self.retract_dc)

    def extend(self):
        self._pwm.ChangeDutyCycle(self.extend_dc)


    def clean(self):
        self._pwm.stop()
        GPIO.cleanup()
