clear 
clc
obj=aircraft();
syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w px py pz Phi Theta Psi p q r
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi px py pz];

u0=11.45;
v0=0;
w0=1.66464;
Theta0=atan2(w0,u0);
Psi0=atan2(v0,u0);
pointOfEquilibrium=PointOfEquilibrium([u0;v0;w0;0;0;0;0;Theta0;Psi0;0;0;100].', ...
    [5.576,0,0,0,0]);

sys=aircraftSystem(states,inputs,[0;0;0]);
round(subs(sys,[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]),4)
[A,B]=Stab(sys,states,inputs,pointOfEquilibrium);
A=double(round(A,4));
B=double(round(B,4));
C=[zeros(3,9) eye(3)];
rank(obsv(A,C));
rank(ctrb(A,B));
% stari initiale in sistem inertial
Ve=[sqrt(sum(pointOfEquilibrium.States(1:3).^2)),0,0];
% controller lienar de stare pe 9 stari
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
Cnew=subs(systemGradient([Rearth2body(-Phi,-Theta,-Psi)*[u;v;w]],states),[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]);
Cnew=double(Cnew(:,1:9));
N=linsolve(Cnew/((-Am+Bm*Km))*Bm,eye(3))

