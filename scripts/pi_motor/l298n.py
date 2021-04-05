import RPi.GPIO as io
# l298n
class Motor():
    """A class to control one side of an L298N dual H bridge motor driver."""
    def __init__(self, en, in1, in2):
        self.encoder = encoder(pin)
        self.in1 = in1
        self.in2 = in2
        all_pins = [en, self.in1, self.in2]
        io.setmode(io.BCM)       
        io.setup(all_pins, io.OUT)
        io.setwarnings(False)

    def __del__(self):
        io.cleanup()
    
    def pwmOut(self, out):        
        # drive motor CW
        io.output(self.in1, out < 0)
        io.output(self.in2, out > 0)
        io.output(self.en, abs(out))

    def stop(self):
        io.output(self.in1, 0)
        io.output(self.in2, 0)

    def cleanup(self):
        io.cleanup()
        io.setmode(io.BCM)

