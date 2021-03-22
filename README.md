<To use the project, kindly give reference to this webpage>

# Autonomous_Breaking_System_project
Autonomous breaking system project - Respond to an on coming obstacle by slowing down or fully breaking the automobile.

Automobile industry is one of the largest industries worldwide having grown by leaps and bounds over the past century. According to some estimates, there are well over 1.4 billion cars worldwide, only to increase in number in the coming future. The current decade saw a phenomenal rise in the production of electrical vehicles and a slight reduction in the production of conventional automobiles. Fueled by the concern on harmful effects of burning fossil fuels, more and more people in the developed countries are switching to electric vehicles. Original equipment manufacturers (OEM) like Tesla, Toyota, Porsche etc, have come up with very efficient and viable models of electric vehicles that are not only attractive, but also environmental friendly.  Electric vehicles differ with conventional automobiles in a way that, they use a DC motor for torque generation and a conventional automobile uses an internal combustion engine. Use of electrical and electronic components opens up a wide plethora of opportunities for innovation, especially in the field of embedded systems. In this project I propose a smart breaking system that detects an obstacle on the road and takes a decision to either slow down or stop the car based on its distance from the obstacle. This makes use of the Timers, Pulse Width Modulation (PWM), DC-Motor control, Analog to Digital converters (ADCs) and UART communication protocol. 

The project makes use of MSP432 microcontroller to implement automatic breaking system by obstacle detection. The basic technology used to accomplish this is Piezoelectricity. Piezoelectricity is the phenomenon by which electrical signals can be used to generate mechanical/ultrasonic signals and vice versa. The ultrasonic signal is transmitted from the transmitter and the echo after striking the obstacle is captured by the receiver. The time taken by the ultrasonic wave to travel the path 'transmitter-obstacle-receiver' is the measure of the obstacle's distance from the sensors. MSP432 microcontroller is used to generate this sequence of events and measure the time duration and relay it in real time on the serial console on the computer. 
Alongside the obstacle detection, the project also implements DC motor control using potentiometer.  Potentiometer is interfaced to MSP432 via the inbuilt ADC module, which can measure the slightest change in the resistance, based on which a corresponding PWM signal is generated. 


The project can be broadly sub-divided into two major functionalities; 
> DC motor control using PWM and,
> Obstacle detection.


DC motor control using PWM

In an electric vehicle, the DC motor is connected to the accelerator peddle. The more force we apply on the peddle, the faster the motor rotor spins. I have implemented a similar design using potentiometer. A potentiometer is a variable resister whose resistance can be varied from 0 to maximum by mechanically rotating its knob. The Potentiometer is connected between Vcc (3.3V) and 0V, and any variation in its resistance produces a voltage drop in between this range.  MSP432 14 bit ADC is used to measure the voltage change.

MSP432 implements a 14 bit SAR ADC which can be configured using the set of dedicated registers.  The output from the ADC is fed directly into TimerA2 CCR[3] register. The Timer_A2 is configured in capture mode and Reset/Set output mode. Whenever the value of ADC crosses the value stored in CCR[3] the output pin is reset. This generates a PWM waveform of duty cycle proportional to the difference between CCR[0] and CCR[3]. The PWM waveform serves as the input to the DC motors.

I have used the HC SR04 sensor for obstacle detection using ultrasonic waves. The HC SR04 provides two kinds of signals - trigger and echo. The trigger is the output probe signal that is emitted from the transmitter. This is when we are supposed to start the timer count. Once the ultrasonic signal is received back after reflection the Echo signal captures it. This is when we stop the timer. The time duration between the transmitter (Trigger) and receiver (Echo) is a measure of how far the obstacle is. When a pulse of 10µsec or more is given to the Trigger pin, 8 pulses of 40 kHz are generated. Immediately, the Echo pin is made high by the control circuitry in the module. The echo pin remains high till it gets the echo signal of the transmitted pulses back. 

 The echo signal is read as port interrupt on rising and falling edge events. During the first interrupt occurrence (identified by rising edge signal), a global counter is cleared. This counter is updated at every timer count interrupt within the Timer ISR. During the second occurrence of Port interrupt for the falling edge of the echo signal, the value in the global counter is measured. This gives the time taken by the ultrasonic signal to travel the 'transmitter-obstacle-receiver' path. Knowing the speed of ultrasonic waves in air, we can easily calculate the distance to the obstacle.
As sound waves travel at 343ms-1 we can calculate the displacement (s) from the sensor to the obstacle with the below formula,


Based on the distance to the obstacle, I have defined four different breaking profiles.

1.	If the distance to the obstacle is within 75cm to 100cm, the speed is reduced to 3/4th of the original speed.

2.	If the distance to the obstacle is within 50cm to 75cm, the speed is reduced to 1/2 of the original speed.

3.	If the distance to the obstacle is within 25cm to 50cm, the speed is reduced to 1/8th of the original speed.

4.	If the distance to the obstacle is less than 25cm, the car is completely stopped for 10 seconds. No amount variation in the potentiometer can energize the motor in this phase.

Once the car resumes, it goes back to the same speed in which it was running prior to meeting the obstacle. The distance to the obstacle is relayed in real time to the serial monitor using UART communication protocol.
