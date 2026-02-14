# 🚗Buggy
An educational project from the University of Applied Sciences in Aachen. A mini metal buggy powered and controlled by an STM32 board.

![C](https://img.shields.io/badge/C-blue?logo=C)
[![STM](https://img.shields.io/badge/STM-%23c2a906?logo=stmicroelectronics&logoColor=black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![License](https://img.shields.io/badge/License-green)](LICENSE)

![Buggy](./buggy.png)![Buggy](./buggy2.png)

## ⚙️Hardware
- STM32F446RE
- 2 x Slow speed motors
- Arduino Motor Shield
- Arduino 9-Axis Motionshield
- Ultrasonic Sensor
- Metal Frame
- 2 x Rubber Tyres
- Powerbank(PSU)
- Breadboard

## 🏁 Project Goals

**Task 1**  
`Square Pattern` — Drive forward in a square

**Task 2**  
`Obstacle Navigation`

01. Forward until obstacle detected  
02. Turn 90° right → forward 3s  
03. Turn 90° left → check obstacle  

    └─ **If blocked**   → repeat step 2  
    └─ **If clear**     → restart from step 1  
    ![Buggy](./drive_pattern.png)

## Workflow


## 📄 License

This project is licensed under Personal License - see the **[LICENSE](LICENSE)** file for details.
