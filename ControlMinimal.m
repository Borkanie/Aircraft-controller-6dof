function [AEd,BEd,CEd,KEmd,Cmd]=ControlMinimal(pointOfOperation)
u0=11.45;
w0=1.66464;
Theta0=atan2(w0,u0);
% operationStates=[pointOfOperation.operationStates(1:3)-pointOfOperation.operatingWinds(1:3),...
%     pointOfOperation.operationStates(4:12)];
[Am,Bm,Cm,Km,C,E,CTF] = ControllerAroundPointOfOperation(pointOfOperation.States, ...
    pointOfOperation.Inputs,pointOfOperation.Winds);
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
E(4,2)=100*E(4,2);
AE=[Am,E(1:9,:);zeros(3,12)];
AEd=eye(12)+Ts*AE;
BE=[Bm;zeros(3,5)];
BEd=Ts*BE;
CEd=[C(1:9,1:9),zeros(9,3)];
%veriuficam daca sunt observabile
rank(obsv(AEd,CEd))
KEmd=[Kmd -Kmd(:,1:3)];
%kalman filter
%KalmanFilterForExtendedLinearSystem(AEd,BEd,CEd,KEmd,PointOfEquilibrium( ...
%    [pointOfOperation.States,pointOfOperation.Winds],pointOfOperation.Inputs,[pointOfOperation.Winds]));

Kss=[Km zeros(5,3)];
statespace=ss(AE-BE*Kss,BE,CTF,zeros(3,5));
H=minreal(zpk(c2d(tf(statespace),Ts)),.15);
z=tf([1 0],1,Ts);
H=H/z;
%Gain matrix
Gm=evalfr(H,1);
%RGA
RGA=rga(Gm);
end
%imposed transfer function
% h11=tf(1,[1 100],Ts);
% h22=tf(1,[1 100],Ts);
% h33=tf(1,[1 100],Ts);
% %decoupled system
% Hdecuplat=[H(1,1),0,0;0,0,H(2,3);0,H(3,2),0;];
% %imposed MIMO
% Hdorit=[h11,0,0;
%     0,0,h33;
%     0,h22,0;];
% %Regulator calculation
% Hr=minreal(-(Hdorit-1)\Hdorit/Hdecuplat,0.1);
% %trying values
% Htest=zpk(minreal(series(Hr,Hdecuplat)/(1+series(Hr,Hdecuplat)),0.1));
% Hr=[Hr;0,0,0;0,0,0];
% Hnew=zpk(minreal(series(Hr,H)/(1+series(Hr,H)),0.1))
% [numReg11,denReg11]=tfdata(Hr(1,1),'v');
% [numReg12,denReg12]=tfdata(Hr(1,2),'v');
% [numReg13,denReg13]=tfdata(Hr(1,3),'v');
% [numReg21,denReg21]=tfdata(Hr(2,1),'v');
% [numReg22,denReg22]=tfdata(Hr(2,2),'v');
% [numReg23,denReg23]=tfdata(Hr(2,3),'v');
% [numReg31,denReg31]=tfdata(Hr(3,1),'v');
% [numReg32,denReg32]=tfdata(Hr(3,2),'v');
% [numReg33,denReg33]=tfdata(Hr(3,3),'v');