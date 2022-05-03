clear 
clc
obj=aircraft();
syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w px py pz Phi Theta Psi p q r
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi px py pz];

u0=11.45;
v0=1.66464;
Theta0=atan2(v0,u0);
pointOfEquilibrium=PointOfEquilibrium([u0;0;v0;0;0;0;0;Theta0;0;0;0;100].', ...
    [5.576,0,0,0,0]);

sys=aircraftSystem(states,inputs,[0;0;0]);
vpa(sys,4);
round(subs(sys,[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]),4)
[A,B]=Stab(sys,states,inputs,pointOfEquilibrium);
A=double(round(A,4));
B=double(round(B,4));
C=[zeros(3,9) eye(3)];
rank(obsv(A,C));
rank(ctrb(A,B));
error=zeros(1,12);
error(1,1)=0.1;
% stari initiale in sistem inertial
Ve=[sqrt(sum(pointOfEquilibrium.States(1:3).^2)),0,0];




%% controller lienar de stare pe 9 stari
Am=double(round(A(1:9,1:9),2));
Bm=double(round(B(1:9,1:5),2));

%verificam controlabilitate
rank(ctrb(Am,Bm))
vpa(eig(Am),2)
%impunem poli
P=[-1.9,-1.8,-3,-2.4,-2.7,-1,-3,-3.1,-2.9];
Km=place(Am,Bm,P)
%verificam rezultat
eig(Am-Bm*Km)
%%  estimator de stare in continuu
%scopul este sa estimam X7 X8 X9
Cm=eye(9);
Cm(4,4)=0;
Cm(5,5)=0;
Cm(6,6)=0;
%verificam observabilitate
rank(obsv(Am,Cm))
errorest=zeros(1,12);
errorest(1,1)=1.1;
ObsPoles=[-.1,-.2,-.3,-.4,-.5,-.6,-.7,-.8,-.9];
L=place(Am',Cm',ObsPoles).'
eig(Am-L*Cm)
%% estimator de stare in discret
%perioada de esantionara
Ts=.1;
%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%verificam observabilitate
rank(obsv(Amd,Cmd))
errorest=zeros(1,12);
errorest(1,1)=1.1;
%observer discret
%impunem poli
ObsPoles=[-2,-2.5,-3,-3.3,-.5,-5,-6.7,-4.7,-5.6];
Ld=place(Amd',Cmd',ObsPoles).';
eig(Amd-Ld*Cmd)
%controller discret
%impunem poli
Q=eye(9);
Q(1:3,:)=1/10*Q(1:3,:);
R=eye(5);
R(1,:)=1/10*R(1,:);

Kmd=lqr(Am-Ld*Cmd,Bm,Q,R);
Kmd(2,:)=Kmd(2,:)/2;
Kmd(3,:)=-Kmd(3,:);
Kmd(3,9)=-Kmd(3,9)*10;
Kmd(3,7)=Kmd(3,7)*10;
Kmd(4,:)=-Kmd(4,:);
Kmd(4,7)=Kmd(4,7)*10;
Kmd(4,9)=Kmd(4,9)*50;
Kmd(4,2)=-Kmd(4,2)
Km
%verificam rezultat

%% estimator de stare in discret
%perioada de esantionara
Ts=.1;
%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%verificam observabilitate
rank(obsv(Amd,Cmd))
errorest=zeros(1,12);
errorest(1,1)=1.1;
%observer discret
%impunem poli
ObsPoles=[-2,-2.5,-3,-3.3,-.5,-5,-6.7,-4.7,-5.6];
Ld=place(Amd',Cmd',ObsPoles).';
eig(Amd-Ld*Cmd)
%controller discret
%impunem poli
Q=eye(9);
Q(1,1)=25;
Q(7,7)=1;
Q(5,5)=25;
Q(6,6)=1;
Q(9,9)=1;
R=eye(5);
R(1,:)=1*R(1,:);
Kmd=lqr(Am,Bm,Q,R)
Km
%verificam rezultat
%eig(Amd-Bmd*Kmd)
