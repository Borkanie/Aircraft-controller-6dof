syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w px py pz Phi Theta Psi p q r
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi px py pz];

u0=11.45;
v0=0;
w0=1.66464;

Theta0=atan2(w0,u0);
Psi0=atan2(v0,u0);
pointOfEquilibrium=PointOfEquilibrium([u0;0;w0;0;0;0;0;Theta0;Psi0;0;0;100].', ...
    [5.576,0,0,0,0]);
syms uw vw ww
disturbances=[uw;vw;ww];
sys2=aircraftSystem(states,inputs,disturbances);
round(subs(sys2,[states inputs disturbances.'],[pointOfEquilibrium.States pointOfEquilibrium.Inputs 0 0 0]),4)
D1=round(subs(systemGradient(sys2,disturbances),[states inputs disturbances.'],[pointOfEquilibrium.States pointOfEquilibrium.Inputs 1 1 1]),4)
D1m=D1(1:9,:);
K2=double(round(pinv(Bm)*D1m,4))
