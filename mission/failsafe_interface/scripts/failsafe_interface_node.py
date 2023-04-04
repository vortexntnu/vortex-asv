import RPi.GPIO as GPIO

if __name__ == "__main__":
    inPIN = 20
    outPIN = 21

    GPIO.setmode(GPIO.BCM)  # Use BCM numbering

    GPIO.setup(inPIN, GPIO.IN)  # Set pin 4 as input
    GPIO.setup(outPIN, GPIO.OUT)  # Set pin 4 as input

    inSig = GPIO.input(inPIN)

    GPIO.output(outPIN, inSig)

    print("Input: ", inSig)
