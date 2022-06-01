clear all
clc
vantRelevant=-1:0.5:1;
%some function that will return all the states around a different
%operating point described solely by angles and wind starting from
%point of equilibrium
[UW,VW,WW]=meshgrid(vantRelevant,vantRelevant,vantRelevant);
S=size(WW);
Atot=[];
Btot=[];
Ctot=[];
Etot=[];
Wtot=[];
for i=1:S(1)
    for j=1:S(1)
        for k=1:S(1)
            po = GetEquilibriumPoint(UW(i),VW(j),WW(k));
            vant=[UW(i),VW(j),WW(k)];
            [po1,Am,Bm,Cm,Km,A,B,C,E] = ControllerAroundPointOfOperation(po.States,po.Inputs,vant.');
            Atot=[Atot A];
            Btot=[Btot B];
            Ctot=[Ctot C];
            Etot=[Etot E];
            Kmtot=[Kmtot Km];
            Wtot=[Wtot vant.'];
        end
    end
end

