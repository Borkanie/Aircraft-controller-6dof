function [answer] = GetEquilibriumPoint(uw,vw,ww)
eqchil=[11.45,0,1.66464,0,0,0,0,0.144371912240994,0,0,0,0];
in=[5.576,0,0,0,0];
states=[eqchil(1:3)+[uw,vw,ww],eqchil(4:12)];
answer=PointOfEquilibrium(states,in);
end