# thrust-vectoring-fin-control
This repository shows how to use MATLAB to simulate a simple thrust-vectoring fin. Clone the repo and run the main script to show the uncontrolled system, the controlled system, and a simulation of the system. 


This uses ODE45 within its implementation of a PD controller with tuned gains to ensure the payload body maintains an upright position in the air. Although simple, the system was able to correct for a random orientation with minimal overshoot and quick response time. This was used to show the validity of a thrust-vectoring design for the payload requirement in the NASA 2024 Student Launch competition. 
