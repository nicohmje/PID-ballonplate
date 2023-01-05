# Using a PID algo and Computer Vision to stabilize a ping-pong ball on a moving plane

![the complete assembly](https://github.com/nicohmje/PID-ballonplate/blob/main/Complete_assembly.jpg?raw=true)

A relatively straightforward project using a Raspberry Pi, three servos and a 3D printed structure.

[SEE IT IN ACTION!](https://www.reddit.com/r/3Dprinting/comments/vbh1qf/3d_printed_all_the_parts_almost_for_this_fun/)

## Principle behind the project: 

1. A Pi camera tracks the position of the ball, using OpenCV. A mask is first applied separating the orange color of the ball from the rest. The mask is then eroded twice, blurred, and finally a 'smallest enclosing circle' command returns the radius and the center position.

2. This information is then filtered by radius. If it is part of the range of radii that the ball is expected to have, the program calculates the error between the position of the ball and the center of the plate. Three errors total are calculated, one for each servos' line of action.

3. These three errors are then input in the three PID Controllers, including past errors and past derivatives. The PID Controllers then output a change of angle for each servo, which is first normalized against computing time to account for small variations, and then sent using AdaFruit's servo library to the motors. 

## Improvements / Failures

My current platform is unstable at best, and requires constant recalibration (Done through the GUI); a more stable platform could be obtained with a better interface between the arms. Ball bearings could be used to further smoothen the motion of the platform, and more drastically three more servos could be added for a full on Stewart platform. At this scale however with this level of precision, the current platform is suitable enough. 

One could also use Dynamixel Servos for a more accurate position control, although this would significantly increase the price of the whole assembly. 

## Parts list : 

- Raspberry Pi 4b 8 gb
- Pi Camera V2
- Adafruit Camera cable extension
- Three DIYMORE Full metal servos
- Waveshare Servo HAT
- Pimoroni Fan Shim
- Three Dexter Cardan Joints (Universal joints)

The rest of the parts are 3D printed. 

Part selection really doesn't matter that much apart from the RPi and the PiCamera. Use what is available to you for a fair price, and adapt the parts to your needs. 

## Want to replicate this project yourself ? 

Great idea! If you need any help, feel free to shoot me a message on reddit (/u/parisiancyclist) or Instagram (@nicohmje). You can also directly contact me through GitHub. 

Happy building!

