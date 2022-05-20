vitezaRelevanta=6.5:16.5;
vitezaUnghiularaRelevanta=-0.3:0.05:0.3;
unhiRelevant=-0.7854:0.1:0.7854;
vantRelevant=-5:0.5:5;
%%
u0=11.45;
v0=1.66464;
Theta0=atan2(v0,u0);
operationStates=[u0;0;v0;0;0;0;0;Theta0;0;0;0;100];
operatingInputs=[5.576,0,0,0,0];
operatingWinds=[0;0;0];
[pointOfEquilibrium,Am,Bm,Cm,Km] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
Ts=0.1;
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%kalman filter
KalmanFilterForLinearSystem(Amd,Bmd,eye(9),Km,pointOfEquilibrium)