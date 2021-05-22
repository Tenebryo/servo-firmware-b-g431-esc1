# Overview

The aim of this document is to provide an overview of the project source code. Most of the code is generated using ST's Motor Control SDK via the Motor Control Workbench and STM32CubeMX. Most of the additional implementation is in the source and header files prefixed with `user_`.

## MCSDK Components

The ST MCSDK handles the low-level HAL and low-level FOC current control and state machine. One of the most convenient parts is that the MCSDK makes pretty good use of the peripherals available, which would otherwise require a lot of R&D and boilerplate to implement. It also implements circuit protection to prevent over-current, over-voltage, and over-temperature events. Having to otherwise implement this yourself would be a huge barrier to entry, and would most likely also result in dead hardware.

## Servo Controller

### Step/Direction Interface

The step/direction interface is mostly implemented using the seemingly all-powerful STM32 timers, which have a convenient encoder mode called "Clock Plus Direction" that steps the timer counter every rising edge of the Clock input, and the direction it steps is dictated by the Direction input. These inputs are then mapped to the pins that are available for GPIO on the B-G431-ESC board. 

### Automatic Motor Characterization (Planned)

I will use the ODrive firmware as a reference for measuring phase resistance and inductance. I also need to implement encoder direction checks, and reverse the direction if needed. I already have code to align the servo loop to the encoder index pulse.

Another planned algorithm is cogging torque characterization and feedforward compensation, which would greatly improve servo performance for motors that have strong heavy cogging.

### Position Control Loop (Initial version complete)

#### PID Control

A simple PID controller that controls the encoder position to a position setpoint using the torque.

#### PIV Control

This controller takes a command of a position and a velocity setpoint (and optionally takes a torque feedforward term, but this is only used for the controllers below). Proportional control output of the position is added to the velocity setpoint, and the output of PI control of the velocity is used as the torque command.

#### Position Filter Control

This controller takes only a position command, which it passes through a second-order filter to calculate feedforward terms for velocity and torque. The filtered position and feedforward velocity and torque are then passed to the PIV controller as the inputs.

#### Step Direction Control

This controller just takes a step/direction input from a peripheral and passes it through the position input filter.

#### Autotuning (Planned)

I don't really know how to do this at the moment, but it is something I definitely want to look into. The motor will have to already be in place in the mechanical system 

## Configuration

I plan on having the controller be configured over the CANBUS interface available on the board, which will allow tuning and interface parameters to be adjusted, as well as starting various routines such as autotuning and motor characterization, when they are implemented.


