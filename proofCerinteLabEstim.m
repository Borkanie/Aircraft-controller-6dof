%kalman proof pt lab
clear
clc
newSys
%%
%kalman filter
KalmanFilterForLinearSystem(Amd,Bmd,Cmd,Kmd,pointOfEquilibrium)
%%
%kalman extins
%impunem poli
Q=eye(9);
Q(1,:)=Q(1,:);
Q(7,:)=2*Q(7,:);
Q(6,:)=1/100*Q(6,:);
Q(7:9,:)=1/2*Q(7:9,:);
Q(9,:)=100*Q(9,:);
R=1/10*eye(5);
R(1,:)=1/5*R(1,:);
R(3,:)=5*R(3,:);
Km1=lqr(Am,Bm,Q,R)
[RExt,QExt]=KalmanFilterExtendedForLinearSystem(jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Km1,pointOfEquilibrium,Amd,Bmd,Ts)
%%
%estimare intrare necunoscuta
syms uw vw ww
sys=aircraftSystem(states,inputs,[uw;vw;ww]);
[E,AEmd,BEmd,CEmd,LEd,vant]=EstimareIntrareNecunoscuta(sys,[uw;vw;ww],jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Kmd,pointOfEquilibrium,Ts);
%%
%decuplare intrare necunoscuta
syms uw vw ww
sys=aircraftSystem(states,inputs,[uw;vw;ww]);
[AEmd2,BEmd2,CEmd2,F,T,K,H,E]=decuplare(sys,[uw;vw;ww],jordanA(1:9,1:9),jordanB(1:9,:), ...
    states(1:9),inputs,Kmd,pointOfEquilibrium,Ts);