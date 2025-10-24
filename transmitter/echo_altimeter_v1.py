#!/usr/bin/env python3

from brping import Ping1D
import serial
import RPi.GPIO as GPIO
import time
import argparse
import sys

# --- GPIO Setup ---
LED_PIN = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

# --- Modem Configuration ---
MODEM_PORT = '/dev/ttyUSB0'
MODEM_BAUD = 9600
TARGET_ADDRESS = 2  # Delphis receiver address A002

def blink_error(on_time, off_time, repeat=3):
    print(f"Error: Blinking LED - {on_time}s ON / {off_time}s OFF")
    for _ in range(repeat):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(on_time)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(off_time)

def main():
    # 5-second delay before starting
    print("Waiting 5 seconds before initialization...")
    time.sleep(5)

    # --- Argument Parser ---
    parser = argparse.ArgumentParser(description="Ping python library example.")
    parser.add_argument(
        '--device',
        action="store",
        required=False,
        type=str,
        default="/dev/serial0",
        help="Ping device port. E.g: /dev/serial0"
    )
    parser.add_argument(
        '--baudrate',
        action="store",
        type=int,
        default=115200,
        help="Ping device baudrate. E.g: 115200"
    )
    parser.add_argument(
        '--udp',
        action="store",
        required=False,
        type=str,
        help="Ping UDP server. E.g: 192.168.2.2:9090"
    )
    args = parser.parse_args()

    if args.device is None and args.udp is None:
        parser.print_help()
        sys.exit(1)

    # --- Initialize Altimeter ---
    myPing = Ping1D()
    try:
        if args.device is not None:
            print(f"Opening {args.device} at {args.baudrate} bps")
            myPing.connect_serial(args.device, args.baudrate)
        elif args.udp is not None:
            host, port = args.udp.split(':')
            myPing.connect_udp(host, int(port))
    except Exception as e:
        print("Altimeter connection failed:", e)
        blink_error(3, 5)
        return False

    if myPing.initialize() is False:
        print("Failed to initialize Ping!")
        blink_error(3, 5)
        return False

    # --- Initialize Delphis Modem Serial ---
    try:
        modem = serial.Serial(MODEM_PORT, MODEM_BAUD, timeout=1)
    except Exception as e:
        print("Failed to open modem port:", e)
        blink_error(5, 10)
        return False

    # --- Main Loop ---
    print("------------------------------------")
    print("Starting Ping...")
    print("Press CTRL+C to exit")
    print("------------------------------------")

    try:
        while True:
            try:
                data = myPing.get_distance()
                if data:
                    dist = data["distance"]
                    conf = data["confidence"]
                    payload = f"{dist},{conf}"
                    length = len(payload)
                    packet = f"$U{TARGET_ADDRESS:03d}{length:02d}{payload}"

                    # LED ON for 0.5s while sending
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    modem.write(packet.encode())
                    print("Sent:", packet)
                    time.sleep(0.5)
                    GPIO.output(LED_PIN, GPIO.LOW)
                    continue  # Skip sleep since delay already occurred
                else:
                    print("Failed to get distance data")
                    raise Exception("No data from altimeter")
            except Exception as e:
                print("Runtime error:", e)
                blink_error(3, 5)
                return False

    except KeyboardInterrupt:
        print("Terminated by user")
    finally:
        GPIO.cleanup()
        modem.close()

    return True

if __name__ == "__main__":
    while True:
        success = main()
        if success:
            break  # Exit if no error
        print("Retrying program in 20 seconds...")
        time.sleep(20)
