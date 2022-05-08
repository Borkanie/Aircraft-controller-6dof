function [xest,P] = KalmanFilterNoCap(Ad, Bd, y, Cd, xpred, u, P, Q, R)
    xpred=Ad*xpred+Bd*u;
    Pred=Ad*P*Ad.'+Q;
    Kn=Pred*Cd.'/(Cd*Pred*Cd.'+R);
    xest=xpred+Kn*(y-Cd*xpred);
    P=(eye(9)-Kn*Cd)*Pred*(eye(9)-Kn*Cd).'+Kn*R*Kn.';
end