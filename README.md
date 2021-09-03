# spl
- test code with framework-spl  
- modified framework-spl to include stm32f1 (code from ST, but file locations little different in platformio)
- framework-spl : forced clock speed to 24MHz for swim_programmer test code

## swim_programmer
- original concept from https://github.com/dimitarm1/SWIM_Programmer 
- modified code to make it work
- output signal timing is currently tied to 24MHz clock speed, needs rework
- is only a basic test to write a flash address, perform a reset etc
- TODO :rework on opencm3 & integrate with black magic probe


# component tests
- various tests to learn about STM32
- using the different frameworks in platformio

## USB
- old stuff from 2019, not sure if this still works on recent platformio
- the absmouse-stm32arduino doesn't build, TODO check
- mbed : not retested, not using mbed for now


# mbed
old stuff from 2019, not sure if this still works on recent platformio