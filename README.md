# IRIS
IRIS Is a high powered model rocket focused on a precise propulsive landing on a custom tower, similar to SpaceX's falcon 9. It will fly on a H45 motor to approximately 700m, guiding itself to a waypoint in the air using aerodynamic fin control. It will then begin a controlled descent where fins attempt to maintain x and y pos. A landing motor (F15) then ignites to kill velocity, as thrust vectoring helps guide it to the tower where it will be caught by two motorised arms.

This project, while ambitous will be split into serveral mid power and high power flights to test subsystems and more. At the heart of the rocket is the amethyst flight computer, along with two extra boards- a compute module 5 and a raspberry pi zero 2 w. The compute module five will run non linear model predicitive control, an insanely powerful control algorithm to determine required X and Y accels to reach the target given fin or tvc limits. The STM32 based flight computer then turns these accelerations into concrete actuator commands. AN ADDED layer of abstraction is present for the fins in the form of field oriented control, which maintains torque to prevent backdriving.

<img width="107" height="572" alt="Screenshot 2026-04-05 at 4 58 04 pm" src="https://github.com/user-attachments/assets/026a6b62-c11a-41f0-a396-f04afa6889c0" />

Pictured above: FULL CAD of IRIS with transparent airframe for visibility

<img width="1196" height="1012" alt="Screenshot 2026-03-22 at 3 32 23 pm" src="https://github.com/user-attachments/assets/2ffe5cc5-b10e-4d4f-8b41-2050adb5adf7" />

Pictured above: AMETHYST rocket flight computer. Note: Silkscreen colour is subject to change.

<img width="454" height="583" alt="Screenshot 2026-03-17 at 5 57 23 pm" src="https://github.com/user-attachments/assets/7fb2242a-7026-47c2-8722-53c3fa7c6602" />

Pictured Above: Final Avionics Bay design on IRIS

# Bill of Materials (BOM)

| Designator | Part Name | Qty | Value / Part Number | Link |
|---|---|---:|---|---|
| C66,C95,C26,C49,C28,C67,C55,C29,C33,C22,C48,C54,C19,C34,C30,C58,C41,C42,C20,C45,C46,C39,C18,C93,C47,C32,C21,C25,C27,C88,C35 | Ceramic Capacitor (C 0402 1005) | 31 | 100n | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=100n%20C_0402_1005Metric) |
| C23,C24,C61,C60 | Ceramic Capacitor (C 0402 1005) | 4 | 24pf | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=24pf%20C_0402_1005Metric) |
| U3 | 9-DoF IMU / Sensor Fusion IC | 1 | BNO085 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=BNO085) |
| J9,J1,J10,J11 | Pin Header | 4 | Conn_01x03_Socket | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x03_Socket) |
| R45,R48,R46,R47,R49,R44 | Resistor (R 0402 1005) | 6 | R | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Resistor%20%28R%200402%201005%29%20R_0402_1005Metric) |
| R21,R14,R22,R17 | Resistor (R 0603 1608) | 4 | 4.7k | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=4.7k%20R_0603_1608Metric) |
| TP13,TP9,TP14,TP6,TP11,TP10,TP16,TP15,TP5,TP8,TP12 | Test Point | 11 | TestPoint | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Test%20Point%20TestPoint_Pad_1.0x1.0mm) |
| URpiZero1,UCM4 | IC / Module (PinSocket 2x20 P2.54mm Vertical SMD) | 2 | ~ | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=IC%20/%20Module%20%28PinSocket%202x20%20P2.54mm%20Vertical%20SMD%29%20PinSocket_2x20_P2.54mm_Vertical_SMD) |
| R10,R18,R28 | Resistor (R 0402 1005) | 3 | 100k | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=100k%20R_0402_1005Metric) |
| Q2,Q4,Q3 | MOSFET | 3 | BSZ100N06LS3_G | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=BSZ100N06LS3_G) |
| C53,C92,C90,C94 | Ceramic Capacitor (C 0402 1005) | 4 | 10u | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10u%20C_0402_1005Metric) |
| R6,R32,R8,R1 | Resistor (R 0402 1005) | 4 | 100Ω | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=100ohm%20R_0402_1005Metric) |
| J14 | Pin Header | 1 | Conn_01x04 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x04) |
| J17 | U.FL RF Connector | 1 | U.FL-R-SMT_01_ | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=U.FL-R-SMT_01_) |
| U5 | Barometric Pressure Sensor | 1 | MS5611-01BA | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=MS5611-01BA) |
| C37 | Ceramic Capacitor (C 0603 1608) | 1 | 10u | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10u%20C_0603_1608Metric) |
| C91,C86,C89 | Ceramic Capacitor (C 0402 1005) | 3 | 22u | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=22u%20C_0402_1005Metric) |
| C84,C85 | Ceramic Capacitor (C 0402 1005) | 2 | 10pf | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10pf%20C_0402_1005Metric) |
| L11,L10,L9 | Power Inductor | 3 | 6.8 µH | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=6.8%20uH%20WE-XHMA_6060) |
| R34,R33 | Resistor (R 0402 1005) | 2 | 22 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=22%20R_0402_1005Metric) |
| J5,J3,J4 | Connector | 3 | Conn_01x02 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x02) |
| Y1,Y3 | Crystal / Oscillator | 2 | 24mhz | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=24mhz%20TSX-3225) |
| C31 | Ceramic Capacitor (C 0402 1005) | 1 | 10uf | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10uf%20C_0402_1005Metric) |
| J6 | JST Connector | 1 | Conn_01x05_Pin | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x05_Pin) |
| IC2 | Module / IC | 1 | 114993390 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=114993390%20114993390) |
| U2 | 3.3V LDO Regulator | 1 | AMS1117-3.3 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=AMS1117-3.3) |
| U4 | 6-axis IMU | 1 | ICM-42688-P | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=ICM-42688-P) |
| U11,U12,U13 | Buck Converter | 3 | TPS54525PWPR | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=TPS54525PWPR) |
| C36,C38,C56 | Ceramic Capacitor (C 0603 1608) | 3 | 100n | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=100n%20C_0603_1608Metric) |
| IC1 | 128Mbit NOR Flash | 1 | W25Q128JVFIQ | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=W25Q128JVFIQ) |
| R15,R2,R7,R9,R11,R5,R29,R12,R27 | Resistor (R 0402 1005) | 9 | 10k | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10k%20R_0402_1005Metric) |
| R3,R4 | Resistor (R 0402 1005) | 2 | 1k5 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=1k5%20R_0402_1005Metric) |
| U1 | STM32H753 MCU | 1 | STM32H753ZIT6 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=STM32H753ZIT6) |
| C3,C2 | Ceramic Capacitor (C 0805 2012) | 2 | 22u | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=22u%20C_0805_2012Metric) |
| C59 | Ceramic Capacitor (C 0402 1005) | 1 | 10n | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10n%20C_0402_1005Metric) |
| Y4 | Crystal / Oscillator | 1 | Crystal | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Crystal%20/%20Oscillator%20Crystal_SMD_2012-2Pin_2.0x1.2mm) |
| IC7 | Connector / Module | 1 | A121-001-T&R | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=A121-001-T%26R) |
| D2 | LED | 1 | LED | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=LED%20LED_0402_1005Metric) |
| C1 | Ceramic Capacitor (C 0402 1005) | 1 | 10uF | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10uF%20C_0402_1005Metric) |
| J7 | Molex Connector | 1 | 532610471 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=532610471%20MOLEX_532610471) |
| C40,C17 | Ceramic Capacitor (C 0402 1005) | 2 | 2.2u | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=2.2u%20C_0402_1005Metric) |
| J18 | USB Connector | 1 | USB_B_Micro | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=USB_B_Micro) |
| U7 | 8-bit Level Translator | 1 | TXS0108EPW | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=TXS0108EPW) |
| FB1 | Ferrite Bead | 1 | 120R | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=120R%20IND_GZ1608_SNL) |
| D3 | Schottky Diode | 1 | BAT54A | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=BAT54A) |
| J12 | JST Connector | 1 | Conn_01x02 | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x02) |
| U10 | GNSS Receiver | 1 | MAX-M10S-00B | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=MAX-M10S-00B) |
| J8 | XT30 Power Connector | 1 | Conn_01x02_Pin | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x02_Pin) |
| R20,R19 | Resistor (R 0603 1608) | 2 | 10k | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=10k%20R_0603_1608Metric) |
| J2 | JST Connector | 1 | Conn_01x02_Pin | [JLCPCB Search](https://jlcpcb.com/parts/componentSearch?searchTxt=Conn_01x02_Pin) |
