from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important
kit.servo[2].set_pulse_width_range(500, 2500) #Very important



kit.servo[0].angle = 120
kit.servo[1].angle = 120
kit.servo[2].angle = 110

