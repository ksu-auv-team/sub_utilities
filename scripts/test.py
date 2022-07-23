import Jetson.GPIO as GPIO

def main():
    output_pin = 18
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    print("It's go time baybee")
    try:
        GPIO.output(output_pin, GPIO.HIGH)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()