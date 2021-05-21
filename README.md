# Servo Control Firmware for B-G431-ESC Dev Board

The B-G431-ESC development kit/board from ST is a tiny, low-cost system that is designed to run advanced motor control algorithms such as FOC. It is possible to use ST tools to generate project templates that take care of the hardware abstractions, current regulation, hardware protection, and other low-level tasks that are otherwise a pretty difficult barrier to entry for motor control. This repository contains firmware that controls the motor in a position loop using an incremental encoder, and that uses the encoder feature of STM32 timers to create a step/direction interface that is a drop-in replacement for standard stepper motor drivers.

## Done

 * Position Control
 * Step/direction input

## TODO

 * Motion Feedforward from step/direction input
 * Automatic motor characterization
 * Automatic servo control tuning
 * CAN interface for configuration

