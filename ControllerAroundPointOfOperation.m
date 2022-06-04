function [pointOfEquilibrium,Am,Bm,Cnew,Km,A,B,C,E,CTF] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
pointOfEquilibrium=PointOfEquilibrium(operationStates, operatingInputs );
syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w Pozx Pozy Pozz Phi Theta Psi p q r
syms uw vw ww
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi Pozx Pozy Pozz];
winds=[uw,vw,ww];
obj=aircraft();
sys=aircraftSystem(states,inputs,winds.');
vpa(sys,4);
round(subs(sys,[states inputs winds],[pointOfEquilibrium.States pointOfEquilibrium.Inputs operatingWinds.']),4)
[A,B]=Stab(sys,states,inputs,pointOfEquilibrium);
A=double(round(subs(A,winds,operatingWinds.'),4));
B=double(round(subs(B,winds,operatingWinds.'),4));
E=double(round(subs(systemGradient(sys,winds),[states inputs winds],[pointOfEquilibrium.States pointOfEquilibrium.Inputs operatingWinds.']),4));
C=[eye(12)];
%C=[zeros(3,9) eye(3)];
rank(obsv(A,C));
rank(ctrb(A,B));
% controller lienar de stare pe 9 stari
Am=double(round(A(1:9,1:9),4));
Bm=double(round(B(1:9,1:5),4));
%impunem poli
P=[-1.9,-1.8,-3,-2.4,-2.7,-1,-3,-3.1,-2.9];

Km=place(Am,Bm,P);
%verificam rezultat
eig(Am-Bm*Km);
Cnew=eye(9);
CTF=subs(systemGradient([Rearth2body(-Phi,-Theta,-Psi)*[u-uw;v-vw;w-ww]],[states(1:9) winds]), ...
    [states inputs winds],[pointOfEquilibrium.States pointOfEquilibrium.Inputs operatingWinds.']);
CTF=double(CTF);
end