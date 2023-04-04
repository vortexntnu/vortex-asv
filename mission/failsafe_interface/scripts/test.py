import RPi.GPIO as GPIO

if __name__ == "__main__":
    outPIN = 11
    inPIN = 13
    sigPIN = 15

    GPIO.setmode(GPIO.BCM)  # Use BCM numbering

    GPIO.setup(inPIN, GPIO.IN)  # Set pin 4 as input
    GPIO.setup(outPIN, GPIO.OUT)  # Set pin 4 as input
    GPIO.setup(sigPIN, GPIO.OUT)

    GPIO.output(outPIN, 1)

    while(True):
        inSig = GPIO.input(inPIN)

        GPIO.output(sigPIN, inSig)

        print("Input: ", inSig)

