function [pointOfEquilibrium,Am,Bm,Cnew,Km] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
pointOfEquilibrium=PointOfEquilibrium(operationStates.', operatingInputs );
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
% controller lienar de stare pe 9 stari
Am=double(round(A(1:9,1:9),4));
Bm=double(round(B(1:9,1:5),4));
%impunem poli
P=[-1.9,-1.8,-3,-2.4,-2.7,-1,-3,-3.1,-2.9];
Km=place(Am,Bm,P)
%verificam rezultat
eig(Am-Bm*Km)
Cnew=subs(systemGradient([Rearth2body(-Phi,-Theta,-Psi)*[u;v;w]],states),[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]);
Cnew=double(Cnew(:,1:9));
end