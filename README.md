# PID-ballonplate
A simple-ish project using a Raspberry Pi and three servos. 

## Principle behind the project: 

1. A Pi camera tracks the position of the ball, using OpenCV. A mask is first applied separating the orange color of the ball from the rest. THe mask is then eroded twice, blurred, and finally a 'smallest enclosing circle' command returns the radius and the center position.

2. This information is then filtered by radius. If it is part of the range of radii that the ball is expected to have, the program calculates the error between the position of the ball and the center of the plate. Three errors total are calculated, one for each servos' line of action.

3. These three errors are then input in the three PID Controllers, including past errors and past derivatives. The PID Controllers then output a change of angle for each servo, which is first normalized against computing time to account for small variations, and then sent using adafruit's servo library to the servos. 

## Parts list : 

- Raspberry Pi 4b 8 gb
- Pi Camera V2
- Adafruit Camera cable extension
- Three DIYMORE Full metal servos
- Waveshare Servo HAT
- Pimoroni Fan Shim
- Three Dexter Cardan Joints (Universal joints)

The rest of the parts are 3D printed. 



