vitezaRelevanta=6.5:16.5;
vitezaUnghiularaRelevanta=-0.3:0.05:0.3;
unhiRelevant=-0.7854:0.1:0.7854;
vantRelevant=-3:0.5:3;
%some function that will return all the states around a different
%operating point described solely by angles and wind starting from
%point of equilibrium
[UW,VW,WW]=meshgrid(vantRelevant,vantRelevant,vantRelevant);
U=zeros(13,13,13);
V=zeros(13,13,13);
W=zeros(13,13,13);
S=size(WW);
for i=1:S(1)
    for j=1:S(1)
        for k=1:S(1)
            pointOfEquilibrium = GetEquilibriumPoint(UW(i),VW(j),WW(k));
        end
    end
end
%%
u0=11.45;
v0=1.66464;
Theta0=atan2(v0,u0);
operationStates=[u0;0;v0;0;0;0;0;Theta0;0;0;0;100];
operatingInputs=[5.576,0,0,0,0];
operatingWinds=[0;0;0];
[pointOfEquilibrium,Am,Bm,Cm,Km] = ControllerAroundPointOfOperation(operationStates,operatingInputs,operatingWinds)
Ts=0.1;
Amd=(eye(9)+Ts*Am);
Bmd=Ts*Bm;
Cmd=Cm;
%kalman filter
KalmanFilterForLinearSystem(Amd,Bmd,eye(9),Km,pointOfEquilibrium)