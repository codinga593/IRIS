# IRIS
IRIS Is a high powered model rocket focused on a precise propulsive landing on a custom tower, similar to SpaceX's falcon 9. It will fly on a H45 motor to approximately 700m, guiding itself to a waypoint in the air using aerodynamic fin control. It will then begin a controlled descent where fins attempt to maintain x and y pos. A landing motor (F15) then ignites to kill velocity, as thrust vectoring helps guide it to the tower where it will be caught by two motorised arms.

This project, while ambitous will be split into serveral mid power and high power flights to test subsystems and more. At the heart of the rocket is the amethyst flight computer, along with two extra boards- a compute module 5 and a raspberry pi zero 2 w. The compute module five will run non linear model predicitive control, an insanely powerful control algorithm to determine required X and Y accels to reach the target given fin or tvc limits. The STM32 based flight computer then turns these accelerations into concrete actuator commands. AN ADDED layer of abstraction is present for the fins in the form of field oriented control, which maintains torque to prevent backdriving.

<img width="107" height="572" alt="Screenshot 2026-04-05 at 4 58 04 pm" src="https://github.com/user-attachments/assets/026a6b62-c11a-41f0-a396-f04afa6889c0" />

Pictured above: FULL CAD of IRIS with transparent airframe for visibility

<img width="1196" height="1012" alt="Screenshot 2026-03-22 at 3 32 23 pm" src="https://github.com/user-attachments/assets/2ffe5cc5-b10e-4d4f-8b41-2050adb5adf7" />

Pictured above: AMETHYST rocket flight computer. Note: Silkscreen colour is subject to change.

<img width="454" height="583" alt="Screenshot 2026-03-17 at 5 57 23 pm" src="https://github.com/user-attachments/assets/7fb2242a-7026-47c2-8722-53c3fa7c6602" />

Pictured Above: Final Avionics Bay design on IRIS

