%% DC motor Control
%% Autor: Yue Wang -TU Berlin
%% 2021 04/05

R=2;          %% Resistor
L=0.5;        %% Inductor
Km=0.1;       %% Torque Constant
Kemf=0.1;     %% Back EMP Voltage Constant
Kf=0.2;       %% Viscous Friction Constant
J=0.02;       %% inertial Load
Td=0;         %% Last Torque
pi=3.14;

%motor TF Function
s=tf('s');
DC_motor=Km/((J*s+Kf)*(R+L*s)+Km^2);
%DC_motor_SS=ss(DC_motor);
[num,den]=tfdata(DC_motor,'v');
P=pole(DC_motor);
p1=P(1);
p2=P(2);

%% control with PID
DC_motor_P=feedback(C*DC_motor,1);     %% C can designed by simulink PID-Block
linearSystemAnalyzer('step',DC_motor_P,0:0.01:5);
linearSystemAnalyzer('step',DC_motor,0:0.01:5);

%% control with Frequenz Analyze

%margin(DC_motor);
[mag,phase,w]=bode(DC_motor,12.6);     %% Phase margin 60°
lag=tf([1,2.05],[1,0.01]);             %% lag Compensator design
C_frequenz=(1/mag)*0.6;
DC_motor_F=feedback(C_frequenz*lag*DC_motor,1);
linearSystemAnalyzer('step',DC_motor_F,0:0.01:5);
%%margin(DC_motor*C_frequenz*lag);