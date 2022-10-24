# Ball and Beam Control System

<br>

<p align="justify">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;To examine the possibility of controlling an inherently unstable and nonlinear system, in this project a ball and beam laboratory setup is designed and an appropriate ball position control system is implemented. The design of the laboratory setup includes the selection of the appropriate electronic and mechanical components, design of mechanical components that are not available, production of designed components and assembling all elements of the setup into a whole. The implementation of the ball position control system involves defining the input and output variables of the designed setup, identification of the mathematical model of the setup, designing a suitable variable estimator and designing a controller that will control the position of the ball on the beam.</p>

<br>


## Project workflow

<br>

<b>Step 1.</b>&nbsp;&nbsp;Designing and 3D printing the needed components
<br>
<p align="center"><img src="images%20for%20GitHub/assembly%20from%20front%20isometry.JPG" width="400px"></p>
<br>

<b>Step 2.</b>&nbsp;&nbsp;Assembling the components into a system
<br>
<p align="center"><img src="images%20for%20GitHub/real%20assembly.JPG" width="540px"></p>
<br>

<b>Step 3.</b>&nbsp;&nbsp;Identifying the motor model
<br>
<p align="center"><img src="images%20for%20GitHub/motor%20identification%20-%20motor%20rotation%20speed.png" width="540px"></p>
<br>

<b>Step 4.</b>&nbsp;&nbsp;Modeling the ball and beam system
<br>
<p align="center"><img src="images%20for%20GitHub/system%20model.png" width="240px"></p>
<br>


<b>Step 5.</b>&nbsp;&nbsp;Designing the both low-pass filtering estimator and extended Kalman filter (EKF)
<br>
<br>

<b>Step 6.</b>&nbsp;&nbsp;Designing the both PD-PD cascade controller and state-space controller
<br>
<br>

<b>Step 7.</b>&nbsp;&nbsp;Modeling the motor static friction
<br>
<p align="center"><img src="images%20for%20GitHub/friction%20identification%20-%20motor%20rotation%20speed.png" width="540px"></p>
<p align="center"><img src="images%20for%20GitHub/friction%20identification%20-%20real%20voltage.png" width="540px"></p>
<br>


<b>Step 8.</b>&nbsp;&nbsp;Implementing the designed estimators and controllers on the Arduino Uno microcontroller
<br>
<br>


<b>Step 9.</b>&nbsp;&nbsp;Analyzing the results for disturbance compensation (shown for EKF and state-space controller)
<br>
<p align="center"><img src="images%20for%20GitHub/state%20space%20Kalman%20control%20-%20ball%20position.png" width="540px"></p>
<p align="center"><img src="images%20for%20GitHub/state%20space%20Kalman%20control%20-%20modeled%20voltage.png" width="540px"></p>
<br>

<b>Step 10.</b>&nbsp;&nbsp;Analyzing the results for reference tracking (shown for EKF and state-space controller)
<br>
<p align="center"><img src="images%20for%20GitHub/state%20space%20tracking%20control%20-%20ball%20position.png" width="540px"></p>
<p align="center"><img src="images%20for%20GitHub/state%20space%20tracking%20control%20-%20modeled%20voltage.png" width="540px"></p>
<br>

<b>Step 11.</b>&nbsp;&nbsp;Choosing the system with state-space controller and EKF due to best results
<br>
<br>


## Run the project on Windows

<br>

<b>Step 1.</b>&nbsp;&nbsp;Clone the repository:
<pre>
cd %HOMEPATH%

git clone https://github.com/Doc1996/ball-and-beam-system
</pre>
<br>

<b>Step 2.</b>&nbsp;&nbsp;Setup the Arduino IDE for using the Arduino Uno microcontroller
<br>
<br>

<b>Step 3.</b>&nbsp;&nbsp;Upload the Arduino project <i>system_control</i> to the microcontroller
<br>
<br>

<b>Optional step</b>&nbsp;&nbsp;Run the Matlab script <i>identify_and_control_model.m</i> and examine results
<br>
<br>

<b>Optional step</b>&nbsp;&nbsp;Run the Matlab script <i>plot_record_data.m</i> and examine results
<br>
<br>


## Run the project on Linux

<br>

<b>Step 1.</b>&nbsp;&nbsp;Clone the repository:
<pre>
cd $HOME

git clone https://github.com/Doc1996/ball-and-beam-system
</pre>
<br>

<b>Step 2.</b>&nbsp;&nbsp;Setup the Arduino IDE for using the Arduino Uno microcontroller
<br>
<br>

<b>Step 3.</b>&nbsp;&nbsp;Upload the Arduino project <i>system_control</i> to the microcontroller
<br>
<br>

<b>Optional step</b>&nbsp;&nbsp;Run the Matlab script <i>identify_and_control_model.m</i> and examine results
<br>
<br>

<b>Optional step</b>&nbsp;&nbsp;Run the Matlab script <i>plot_record_data.m</i> and examine results
<br>
<br>