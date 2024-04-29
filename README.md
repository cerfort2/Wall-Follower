# Wall-Follower

This repository was created to make a robot follow a walled-off track and publish race data
to the cloud.
Some aspects of the design:
-Bluetooth-enabled starting and stopping
-Automatic wall detection and driving
-Automatic detection of finish line
-MQTT race data publishing
To run the code, you need to first make sure the directory includes the following files from
previous labs:
- driverlib.c
- simplelink.c
- sl_common.c
- MQTTClient.c
- Reflectance.c
- Clock.c
- Bump.c
- Motor.c
- I2CB1.c
- CortexM.c
- LPF.c
- opt3101.c
- LaunchPad.c
- UART0.c
- SSD1306.c
- FFT.c
- SysTickInts.c
## Once these are in the working directory, flash main.c onto the board and open a PuTTy
terminal and configure it to read serially from the bot. From the PuTTy output, the user
should see when the bot has successfully connected to the web-app and is ready to
publish. From here, the bot should start following a wall and will stop after running over a white sheet of paper.
