function [pointOfEquilibrium,Am,Bm,Cnew,Km,A,B,C] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
pointOfEquilibrium=PointOfEquilibrium(operationStates.', operatingInputs );
syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w Pozx Pozy Pozz Phi Theta Psi p q r
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi Pozx Pozy Pozz];
obj=aircraft();
sys=aircraftSystem(states,inputs,operatingWinds);
vpa(sys,4);
round(subs(sys,[states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]),4)
[A,B]=Stab(sys,states,inputs,pointOfEquilibrium);
A=double(round(A,4));
B=double(round(B,4));
C=[eye(12)];
%C=[zeros(3,9) eye(3)];
rank(obsv(A,C));
rank(ctrb(A,B));
% controller lienar de stare pe 9 stari
Am=double(round(A(1:9,1:9),4));
Bm=double(round(B(1:9,1:5),4));
%impunem poli
P=[-1.9,-1.8,-3,-2.4,-2.7,-1,-3,-3.1,-2.9];

Km=place(Am,Bm,P)
%verificam rezultat
eig(Am-Bm*Km)
Cnew=eye(9);
% Cnew=subs(systemGradient([Rearth2body(-Phi,-Theta,-Psi)*[u;v;w]],states), ...
%     [states inputs],[pointOfEquilibrium.States pointOfEquilibrium.Inputs]);
% Cnew=double(Cnew(:,1:12));
end