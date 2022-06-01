u0=11.45;
w0=1.66464;
Theta0=atan2(w0,u0);
operatingInputs=[5.576,0,0,0,0];
operatingWinds=[1;1;1];
operationStates=[u0+operatingWinds(1);0+operatingWinds(2);w0+operatingWinds(3);0;0;0;0;Theta0;0;0;0;100].';
[pointOfEquilibrium,Am,Bm,Cm,Km,A,B,C,E] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
Ts=0.1;
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
Q=eye(9);
Q(4:6,:)=1/20*Q(4:6,:);
R=5*eye(5);
R(1,:)=R(1,:)/20;
R(3,:)=5*R(3,:);
Kmd=dlqr(Amd,Bmd,Q,R);
%kalman filter
%KalmanFilterForLinearSystem(Amd,Bmd,Cm,Kmd,pointOfEquilibrium)
%extindere stare perturbatie
E(4,2)=100*E(4,2);
AE=[Am,E(1:9,:);zeros(3,12)];
AEd=eye(12)+Ts*AE;
BE=[Bm;zeros(3,5)];
BEd=Ts*BE;
CEd=[C(1:9,1:9),zeros(9,3)];
%veriuficam daca sunt observabile
rank(obsv(AEd,CEd))
KEmd=[Kmd zeros(5,3)];
%kalman filter
KalmanFilterForExtendedLinearSystem(AEd,BEd,CEd,KEmd,PointOfEquilibrium([pointOfEquilibrium.States,1,1,1],pointOfEquilibrium.Inputs))