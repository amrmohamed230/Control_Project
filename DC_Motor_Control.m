%For the armature-controlled DC motor considering the motor position as the output. Use the
%given values for the parameters of the system in the table and obtain the following:
%Parameter                Values 
%Armature Resistance      1.17 Ohm
%Armature Inductance      24 m.H 
%Shaft Friction           0.0737 N.m.s 
%Shaft Inertia            0.01 Kg.m^2 
%Motor Constant           0.072 V / (rad/sec) 
% a) Get the open loop transfer function of the position control system.
%Motor Equations:
% 1- Electrical Equation:
% ea(t)=Ra*ia(t)+La*dia(t)/dt+Vb, where Vb=Km*Wm(t), Tm(t)=Km*ia(t) and
% thetam(t)=dWm(t)/dt
% Ea(s)=(Ra/Km)*Tm(s)+(La/Km)*s*Tm(s)+Km*s*thetam(s) ----(1)
% 2- Mechanical Equation:
% Tm(t)=Jm*dWm(t)/dt+Dm*Wm(t)
% Tm(s)=Jm*(s^2)*thetam(s)+Dm*s*thetam(s) ----(2)
% from (2) in (1)
% Ea(s)=[(Ra/Km)+(La/Km)*s]*[Jm*(s^2)*thetam(s)+Dm*s*thetam(s)]+Km*s*thetam(s)
% Ea(s)=([(Ra/Km)+(La/Km)*s]*[Jm*(s^2)+Dm*s]+Km*s)*thetam(s)
% By substituting by the above parameters:
% Ea(s)=([16.25+s/3]*[0.01(s^2)+0.0737*s]+0.072*s)*thetam(s)
% Ea(s)=[0.0033*s^3+0.1871*s^2+1.2696*s]*thetam(s)
% T.F=thetam(s)/Ea(s)=1/[0.0033*s^3+0.1871*s^2+1.2696*s]
% b) Considering unity feedback and gain controller "k", draw the root-locus of the system.
num=[1];
den=[0.0033 0.1871 1.2696 0];
G=tf(num,den);
%rlocus(G);
% c) Get the range of the controller gain to ensure system stability.
%sisotool(G)
%The system is stable if K<71.98
% d) Get the controller gain to satisfy (if exists)
%i. Damping ratio equals to 0.7
% K=4.367
%ii. Settling time less than 0.75 sec, 2 sec
% less than 0.75s >> Ts<0.75s is not achievable
% less than 2s >> K,31
%iii. Natural Frequency equals 5 rad/sec.
% K=4.0813
% e) Get the controller gain to achieve critically damped response
% K=2.304
% f) alculate the error steady state error for unit step and unit ramp input using this gain controller
% in (e).
% for step input:
Kp=dcgain(2.304*G)
% Kp=inf
Ess=1/(1+Kp)
% Ess=0
% for ramp input:
% Ess=1/Kv, Kv=lim<s=0>(s*G)
G1=tf(conv(num,[1 0]),den);
Kv=dcgain(2.304*G1)
Ess1=1/Kv
% Kv=1.8147
% Ess=55.1%
% g) Assume that the system is cascaded by a controller 𝐶(𝑠) = 7*(s+15)/(s+25) 
% get the system settling time, overshoot, and error steady state
% from sisotool
% Ts=0.705 sec
% O.S = 1.58%
% for step input: Ess=1/1+Kp
G2=tf(conv(num,[1 15]),conv(den,[1 25]));
Kp2=dcgain(7*G2)
Ess2=1/(1+Kp2)
%Ess=0