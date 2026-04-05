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
|---|---:|---:|---|---|
| C66,C95,C26,C49,C28,C67,C55,C29,C33,C22,C48,C54,C19,C34,C30,C58,C41,C42,C20,C45,C46,C39,C18,C93,C47,C32,C21,C25,C27,C88,C35 | Ceramic Capacitor | 31 | 100nF, 0402, Samsung CL05B104KO5NNNC (JLC C1525) | [Link](https://jlcpcb.com/partdetail/1877-CL05B104KO5NNNC/C1525) |
| C23,C24,C61,C60 | Ceramic Capacitor | 4 | 24pF, 0402, C0G/NP0, FH 0402CG240J500NT (LCSC C1553) | [Link](https://www.lcsc.com/product-detail/C1553.html) |
| U3 | IMU | 1 | BNO085 (JLC C5189642) | [Link](https://jlcpcb.com/partdetail/CEVA-BNO085/C5189642) |
| J9,J1,J10,J11 | Socket Header | 4 | 1x03, 2.54mm, vertical, Amphenol SIP050-1X03-157BLF | [Link](https://www.digikey.com/en/products/detail/amphenol-cs-fci/SIP050-1X03-157BLF/1541044) |
| R45,R48,R46,R47,R49,R44 | Resistor | 6 | TBD - source BOM only says 'R' (exact value not specified) |  |
| R21,R14,R22,R17 | Resistor | 4 | 4.7kΩ, 0603, UNI-ROYAL 0603WAF4701T5E (LCSC C23162) | [Link](https://www.lcsc.com/product-detail/C23162.html) |
| TP13,TP9,TP14,TP6,TP11,TP10,TP16,TP15,TP5,TP8,TP7 | Test Point | 11 | PCB test pad 1.0x1.0mm - no discrete purchasable part specified |  |
| URpiZero1,UCM4 | Socket Header | 2 | 2x20 Raspberry Pi socket/riser header, Adafruit 4079 | [Link](https://www.digikey.com.au/en/products/detail/adafruit-industries-llc/4079/9858462) |
| R10,R18,R28 | Resistor | 3 | 100kΩ, 0402, UNI-ROYAL 0402WGF1003TCE (JLC/LCSC C25741) | [Link](https://www.lcsc.com/product-detail/C25741.html) |
| Q2,Q4,Q3 | MOSFET | 3 | BSZ100N06LS3 G, Infineon | [Link](https://www.digikey.com/en/products/detail/infineon-technologies/BSZ100N06LS3GATMA1/1936392) |
| C53,C92,C90,C94 | Ceramic Capacitor | 4 | 10uF, 0402, Samsung CL05A106MQ5NUNC (JLC C15525) | [Link](https://jlcpcb.com/partdetail/16204-CL05A106MQ5NUNC/C15525) |
| R6,R32,R8,R1 | Resistor | 4 | 100Ω, 0402, Ever Ohms CR0402F100RQ10 (JLC C429511) | [Link](https://jlcpcb.com/partdetail/Ever_OhmsTech-CR0402F100RQ10/C429511) |
| J14 | Pin Header | 1 | 1x04, 2.54mm, vertical, generic 4-pin header | [Link](https://newhavendisplay.com/1x4-single-row-pin-header-connector/) |
| J17 | RF Connector | 1 | Hirose U.FL-R-SMT(01) | [Link](https://www.digikey.com.au/en/products/detail/hirose-electric-co-ltd/U-FL-R-SMT-01/513010) |
| U5 | Barometer | 1 | MS5611-01BA03 / MS561101BA03-50 (JLC C15639) | [Link](https://jlcpcb.com/partdetail/TEConnectivity-MS561101BA0350/C15639) |
| C37 | Ceramic Capacitor | 1 | 10uF, 0603, Samsung CL10A106MA8NRNC (LCSC C96446) | [Link](https://www.lcsc.com/product-detail/C96446.html) |
| C91,C86,C89 | Ceramic Capacitor | 3 | 22uF, 0402, JLCPCB Assembly 22uF/16V (JLC C9900105298) | [Link](https://jlcpcb.com/partdetail/JLCPCBAssembly-22uF16V/C9900105298) |
| C84,C85 | Ceramic Capacitor | 2 | 10pF, 0402, FH 0402CG100J500NT (JLC C1545) | [Link](https://jlcpcb.com/partdetail/1897-0402CG100J500NT/C1545) |
| L11,L10,L9 | Inductor | 3 | 6.8uH, Würth WE-XHMA 6060 | [Link](https://www.mouser.com/ProductDetail/Wurth-Elektronik/74437368068) |
| R34,R33 | Resistor | 2 | 22Ω, 0402, YAGEO RC0402FR-0722RL (LCSC C114765) | [Link](https://www.lcsc.com/product-detail/C114765.html) |
| J5,J3,J4 | Terminal Block | 3 | Same Sky / CUI TB002-500-02BE | [Link](https://www.digikey.com.au/en/products/detail/same-sky-formerly-cui-devices/TB002-500-02BE/10064069) |
| Y1,Y3 | Crystal | 2 | 24MHz, 3225, Suzhou Liming 3225-24.00-20-10-10/A (JLC C518165) | [Link](https://jlcpcb.com/partdetail/534702-3225_24_00_20_10_10A/C518165) |
| C31 | Ceramic Capacitor | 1 | 10uF, 0402, Samsung CL05A106MQ5NUNC (JLC C15525) | [Link](https://jlcpcb.com/partdetail/16204-CL05A106MQ5NUNC/C15525) |
| J6 | JST Header | 1 | JST B5B-XH-A, 5-pin, 2.50mm, vertical | [Link](https://www.digikey.com/en/products/detail/jst-sales-america-inc/B5B-XH-A/1530483) |
| IC2 | LoRa Module | 1 | Seeed 114993390 / Wio-SX1262 | [Link](https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/114993390/25835823) |
| U2 | LDO Regulator | 1 | AMS1117-3.3, SOT-223 (JLC C6186) | [Link](https://jlcpcb.com/partdetail/Advanced_MonolithicSystems-AMS1117_33/C6186) |
| U4 | IMU | 1 | ICM-42688-P (JLC C1850418) | [Link](https://jlcpcb.com/partdetail/TDKInvenSense-ICM_42688P/C1850418) |
| U11,U12,U13 | Buck Converter | 3 | TPS54525PWPR (JLC C140350) | [Link](https://jlcpcb.com/partdetail/TexasInstruments-TPS54525PWPR/C140350) |
| C36,C38,C56 | Ceramic Capacitor | 3 | 100nF, 0603, YAGEO CC0603KRX7R9BB104 (LCSC C14663) | [Link](https://www.lcsc.com/product-detail/C14663.html) |
| IC1 | NOR Flash | 1 | W25Q128JVFIQ (JLC C111478) | [Link](https://jlcpcb.com/partdetail/WinbondElec-W25Q128JVFIQ/C111478) |
| R15,R2,R7,R9,R11,R5,R29,R12,R27 | Resistor | 9 | 10kΩ, 0402, UNI-ROYAL 0402WGF1002TCE (JLC C25744) | [Link](https://jlcpcb.com/partdetail/26487-0402WGF1002TCE/C25744) |
| R3,R4 | Resistor | 2 | 1.5kΩ, 0402, UNI-ROYAL 0402WGF1501TCE (LCSC C25867) | [Link](https://www.lcsc.com/product-detail/C25867.html) |
| U1 | MCU | 1 | STM32H753ZIT6 (JLC C730207) | [Link](https://jlcpcb.com/partdetail/STMicroelectronics-STM32H753ZIT6/C730207) |
| C3,C2 | Ceramic Capacitor | 2 | 22uF, 0805, Samsung CL21A226MAYNNNE (LCSC C45783) | [Link](https://www.lcsc.com/product-detail/C45783.html) |
| C59 | Ceramic Capacitor | 1 | 10nF, 0402, Kyocera AVX 0402YC103K4T2A (JLC C2172121) | [Link](https://jlcpcb.com/partdetail/KyoceraAVX-0402YC103K4T2A/C2172121) |
| Y4 | Crystal | 1 | TBD - source BOM says only 'Crystal' in 2012 package |  |
| IC7 | Radar Sensor | 1 | Acconeer A121-001-T&R / A121 | [Link](https://www.mouser.com/ProductDetail/Acconeer/A121-001-TR) |
| D2 | LED | 1 | TBD - source BOM says only 'LED' in 0402 package |  |
| C1 | Ceramic Capacitor | 1 | 10uF, 0402, Samsung CL05A106MQ5NUNC (JLC C15525) | [Link](https://jlcpcb.com/partdetail/16204-CL05A106MQ5NUNC/C15525) |
| J7 | Molex PicoBlade Header | 1 | Molex 53261-0471 | [Link](https://www.digikey.com/en/products/detail/molex/0532610471/699096) |
| C40,C17 | Ceramic Capacitor | 2 | 2.2uF, 0402, Samsung CL05A225MQ5NSNC (JLC C12530) | [Link](https://jlcpcb.com/partdetail/13164-CL05A225MQ5NSNC/C12530) |
| J18 | USB Connector | 1 | Würth 629105150521, Micro USB B | [Link](https://www.digikey.com.au/en/products/detail/w%C3%BCrth-elektronik/629105150521/5047751) |
| U7 | Level Shifter | 1 | TXS0108EPWR (JLC C17206) | [Link](https://jlcpcb.com/partdetail/TexasInstruments-TXS0108EPWR/C17206) |
| FB1 | Ferrite Bead | 1 | Sunlord GZ1608D601TF (JLC C1002) | [Link](https://jlcpcb.com/partdetail/Sunlord-GZ1608D601TF/C1002) |
| D3 | Schottky Diode | 1 | BAT54A, SOT-23 (JLC C397609) | [Link](https://jlcpcb.com/partdetail/AnBon-BAT54A/C397609) |
| J12 | JST Header | 1 | JST S2B-XH-A, 2-pin, 2.50mm, right-angle | [Link](https://www.digikey.com/en/products/detail/jst-sales-america-inc/S2B-XH-A/1651055) |
| U10 | GNSS Module | 1 | u-blox MAX-M10S-00B (JLC C4153167) | [Link](https://jlcpcb.com/partdetail/UBLOX-MAX_M10S00B/C4153167) |
| J8 | XT30 Connector | 1 | AMASS XT30U-M (JLC C99101) | [Link](https://jlcpcb.com/partdetail/Changzhou_AmassElec-XT30UM/C99101) |
| R20,R19 | Resistor | 2 | 10kΩ, 0603, YAGEO RC0603FR-0710KL (LCSC C98220) | [Link](https://www.lcsc.com/product-detail/C98220.html) |
| J2 | JST Header | 1 | JST B2B-XH-A, 2-pin, 2.50mm, vertical | [Link](https://www.digikey.com/en/products/detail/jst-sales-america-inc/B2B-XH-A/1651045) |
