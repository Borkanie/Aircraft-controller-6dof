function [answer] = GetEquilibriumPoint(uw,vw,ww)

syms T DeltaSt DeltaRud DeltaAlL DeltaAlR
syms u v w Phi Theta Psi p q r PozX PozY PozZ
inputs=[T,DeltaSt,DeltaRud,DeltaAlL,DeltaAlR];
states=[u v w p q r Phi Theta Psi PozX PozY PozZ];
vantRelevant=[uw,vw,ww].';
sys=aircraftSystem(states,inputs,vantRelevant);
state=[11.45
0
1.66464
0
0
0
0
0.144371912240994
0];
in=[5.576
0
0
0
0];
symuri=[inputs,states(1:9)];
sys=subs(sys,inputs,[5.5,0,0,0,0]);
eqn=[sys==[zeros(12,1)]];
answer=solve(eqn,[in.',state.'],symuri.')

end