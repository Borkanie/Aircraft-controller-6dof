function [xest,P,Ad,Bd] = KalmanFilterExtendedCox(jordanA,jordanB, states, inputs, xest,xpred, u, P, Q, Cd, R, y,Ts,punctDeEchilibru)
    intrari=u;
    intrari(2)=getAngle(intrari(2));
    intrari(3)=getAngle(intrari(3));
    intrari(4)=getAngle(intrari(4));
    intrari(5)=getAngle(intrari(5));
    Anew=subs(jordanA,[states inputs],[xest.'+punctDeEchilibru.States.' intrari.']);
    Bnew=subs(jordanB,[states inputs],[xest.'+punctDeEchilibru.States.' intrari.']);
    %liniarizam in jurul noilui punct de echilibri
    Ad=eye(9)+Ts*double(round(Anew,4));
    Bd=Ts*double(round(Bnew,4));
    %recaluculam greutatea predictiei
    Pred=Ad*P*Ad.'+Q;
    %recalculam castig kalman
    Kn=Pred*Cd.'/(Cd*Pred*Cd.'+R);
    %facem estimare
    xest=xpred+Kn*(y-Cd*(xpred+punctDeEchilibru.States));
    %actualizam P
    P=(eye(9)-Kn*Cd)*Pred*(eye(9)-Kn*Cd).'+Kn*R*Kn.';
end