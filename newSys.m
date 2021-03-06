clear 
clc


u0=11.45;
v0=1.66464;
Theta0=atan2(v0,u0);
operationStates=[u0;0;v0;0;0;0;0;Theta0;0;0;0;100];
operatingInputs=[5.576,0,0,0,0];
operatingWinds=[0;0;0];

pointOfEquilibrium=PointOfEquilibrium(operationStates.', operatingInputs, [0,0,0]);
syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w px py pz Phi Theta Psi p q r
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi px py pz];
obj=aircraft();
sys=aircraftSystem(states,inputs,operatingWinds);
vpa(sys,4);
round(subs(sys,[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]),4)
[A,B]=Stab(sys,states,inputs,pointOfEquilibrium);
jordanA=systemGradient(sys,states);
jordanB=systemGradient(sys,inputs);
A=double(round(A,4));
B=double(round(B,4));
C=[zeros(3,9) eye(3)];
rank(obsv(A,C));
rank(ctrb(A,B));
error=zeros(1,12);
error(1,1)=0.1;
% stari initiale in sistem inertial
Ve=[sqrt(sum(pointOfEquilibrium.States(1:3).^2)),0,0];
%perioada de esantionara
Ts=.1;



%% controller lienar de stare pe 9 stari
Am=double(round(A(1:9,1:9),4));
Bm=double(round(B(1:9,1:5),4));

%verificam controlabilitate
rank(ctrb(Am,Bm))
vpa(eig(Am),2)
%impunem poli
P=[-1.9,-1.8,-3,-2.4,-2.7,-1,-3,-3.1,-2.9];
Km=place(Am,Bm,P)
%verificam rezultat
eig(Am-Bm*Km)
Cnew=subs(systemGradient([Rearth2body(-Phi,-Theta,-Psi)*[u;v;w]],states),[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]);
Cnew=double(Cnew(:,1:9));

%%  estimator de stare in continuu
%scopul este sa estimam X7 X8 X9
Cm=eye(9);
%verificam observabilitate
rank(obsv(Am,Cm))
errorest=zeros(1,12);
errorest(1,1)=1.1;
ObsPoles=[-1.15,-1.25,-1.35,-2.45,-3.5,-3.6,-3.7,-3.8,-5];
L=place(Am',Cm',ObsPoles).'
eig(Am-L*Cm)
%% estimator de stare in discret cu control pe lienar

%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%verificam observabilitate
rank(obsv(Amd,Cmd))

ObsPoles=[-2,-2.5,-3,-3.3,-.5,-5,-6.7,-4.7,-5.6]*Ts;
Ld=place(Amd',Cmd',ObsPoles).';
eig(Amd-Ld*Cmd)
%controller discret
%impunem poli
Q=eye(9);
Q(1,:)=Q(1,:);
Q(7:9,:)=1/10*Q(7:9,:);
R=eye(5);
R(1,:)=R(1,:);
Kmd=dlqr(Amd,Bmd,Q,R)
%% estimator de stare in discret cu control pe nelienar
%perioada de esantionara
Ts=.1;
%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%verificam observabilitate
rank(obsv(Amd,Cmd))

%observer discret
%impunem poli
ObsPoles=[-.2,-.55,-.8,-.33,-.5,-.5,-.67,-.47,-.56]*10;
Ld=place(Amd',Cmd',ObsPoles).';
eig(Amd-Ld*Cmd)
%controller discret
%controller discret
%impunem poli
Q=eye(9);
Q(1,:)=2*Q(1,:);
Q(2,:)=2*Q(2,:);
Q(3,:)=2*Q(3,:);
Q(4:6,:)=5*Q(4:6,:);
R=1/15*eye(5);
R(1,:)=1/5*R(1,:);
Kmdnl=lqr(Am-Ld*Cmd,Bm,Q,R)


