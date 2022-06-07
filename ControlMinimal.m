u0=11.45;
w0=1.66464;
Theta0=atan2(w0,u0);
operatingInputs=[5.576,0,0,0,0];
operatingWinds=[1;1;1];
operationStates=[u0+operatingWinds(1);0+operatingWinds(2);w0+operatingWinds(3);0;0;0;0;Theta0;0;0;0;100].';
[pointOfEquilibrium,Am,Bm,Cm,Km,A,B,C,E,CTF] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
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
KalmanFilterForExtendedLinearSystem(AEd,BEd,CEd,KEmd,PointOfEquilibrium([pointOfEquilibrium.States,1,1,1],pointOfEquilibrium.Inputs,[0,0,0]))

Kss=[Km zeros(5,3)];
statespace=ss(AE-BE*Kss,BE,CTF,zeros(3,5));
H=minreal(zpk(c2d(tf(statespace),Ts)),.15);
z=tf([1 0],1,Ts);
H=H/z;
%output/input
%Gain matrix
Gm=evalfr(H,1);
RGA=rga(Gm)
h11=tf(1,[1 100],Ts);
h22=tf(1,[1 100],Ts);
h33=tf(1,[1 100],Ts);
Hdecuplat=[H(1,1),0,0;0,0,H(2,3);0,H(3,2),0;];
% Hr11=h11/H(1,1)*1/(1-h11);
% Hr22=h22/H(3,2)*1/(1-h22);
% Hr33=h33/H(2,3)*1/(1-h33);
% Hr=[Hr11,0,0;
%     0,Hr22,0;
%     0,0,Hr33];
Hdorit=[h11,0,0;
    0,0,h33;
    0,h22,0;];
Hr=minreal(-(Hdorit-1)\Hdorit/Hdecuplat,0.1);
Htest=zpk(minreal(series(Hr,Hdecuplat)/(1+series(Hr,Hdecuplat)),0.1));
Hr=[Hr;0,0,0;0,0,0];
Hnew=zpk(minreal(series(Hr,H)/(1+series(Hr,H)),0.1))
[numReg11,denReg11]=tfdata(Hr(1,1),'v');
[numReg12,denReg12]=tfdata(Hr(1,2),'v');
[numReg13,denReg13]=tfdata(Hr(1,3),'v');
[numReg21,denReg21]=tfdata(Hr(2,1),'v');
[numReg22,denReg22]=tfdata(Hr(2,2),'v');
[numReg23,denReg23]=tfdata(Hr(2,3),'v');
[numReg31,denReg31]=tfdata(Hr(3,1),'v');
[numReg32,denReg32]=tfdata(Hr(3,2),'v');
[numReg33,denReg33]=tfdata(Hr(3,3),'v');