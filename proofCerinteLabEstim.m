%kalman proof pt lab
clear
clc
newSys

%%
%kalman filter

KalmanFilterForLinearSystem(Amd,Bmd,Cmd,Kmd,pointOfEquilibrium)
%%
%kalman extins
KalmanFilterExtendedForLinearSystem(jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Kmd,pointOfEquilibrium,Amd,Bmd,Ts)

%%
%estimare intrare necunoscuta
syms uw vw ww
sys=aircraftSystem(states,inputs,[uw;vw;ww]);
[AEmd,BEmd,CEmd,LEd]=EstimareIntrareNecunoscuta(sys,[uw;vw;ww],jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Kmd,pointOfEquilibrium,Ts);
%%
%decuplare intrare necunoscuta
syms uw vw ww
sys=aircraftSystem(states,inputs,[uw;vw;ww]);
decuplare(sys,[uw;vw;ww],jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Kmd,pointOfEquilibrium,Ts);