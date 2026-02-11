#!/usr/bin/env python3
"""
Software PWM servo test for Pin 15 on Jetson Orin Nano.
Uses a compiled C program for precise bit-bang timing via gpio character device.
Run as root: sudo python3 /tmp/sw_servo_test.py
"""
import subprocess, time, os, sys

CHIP = 0
LINE = 85  # Pin 15 = gpiochip0, line 85

# Kill existing GPIO users and disable hardware PWM
subprocess.run(['pkill', '-f', f'gpioset.*{LINE}='], capture_output=True)
try:
    with open('/sys/class/pwm/pwmchip0/pwm0/enable', 'w') as f:
        f.write('0')
except:
    pass

print('=== Software PWM Servo Test on Pin 15 ===')
print('If servo moves: wiring OK, hardware PWM routing is the issue')
print('If servo does NOT move: check wiring (signal=Pin15, VCC=Pin2/4, GND=Pin14)')
print()

# Compile the C bit-bang program
src = '/tmp/servo_test.c'
binary = '/tmp/servo_test'

if not os.path.exists(src):
    print(f'ERROR: {src} not found. SCP it from the host first.')
    sys.exit(1)

result = subprocess.run(['gcc', '-o', binary, src], capture_output=True, text=True)
if result.returncode != 0:
    print(f'Compile error: {result.stderr}')
    sys.exit(1)

print('Compiled OK. Starting servo sweep...')
print()

# Test: center, left, right, center
positions = [
    (1500, '90 deg CENTER'),
    (700, '~20 deg LEFT'),
    (2300, '~160 deg RIGHT'),
    (1500, '90 deg CENTER'),
]

for pulse, label in positions:
    print(f'  -> {label} (pulse={pulse}us)')
    result = subprocess.run([binary, str(pulse), '2'], capture_output=True, text=True, timeout=10)
    if result.stdout.strip():
        print(f'     {result.stdout.strip()}')
    time.sleep(0.3)

print()
print('Sweep complete.')
