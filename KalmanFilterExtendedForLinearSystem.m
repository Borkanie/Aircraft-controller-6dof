function [] = KalmanFilterExtendedForLinearSystem(sys,states,inputs,Kd,x0,xest0,punctDeEchilibru)
R=eye(9)*10;
Q=eye(9);
%rpedictie de precizie intiala
P=eye(9);
%numar de iterati
iter=5;
Cd=eye(9);
x=x0;
xest=xest0;
xpred=zeros(9,1);
for n=2:iter
    %calculam intrare
    u(:,n-1)=-Kd*x(:,n-1);
    %generam zgomot de proces
    process_noise=(rand(9,1)-rand(9,1))*.5;
    %calculam valoare actuala proces
    val=double(subs(sys,[states inputs],[x(:,n-1).'+punctDeEchilibru.States(1:9),u(:,n-1).'+punctDeEchilibru.Inputs]));
    x(:,n)=val+process_noise;
    %generam zgomot de citire
    reading_noise=(rand(9,1)-rand(9,1))*.5;
    %facem citire
    y(:,n)=x(:,n)+reading_noise+punctDeEchilibru.States(1:9).';
    %facem predictie
    xpred(:,n)=subs(sys,[states inputs], ...
        [punctDeEchilibru.States(1:9)+x(:,n-1).' punctDeEchilibru.Inputs+u(:,n-1).'])+punctDeEchilibru.States(1:9).';
    %liniarizam in jurul noilui punct de echilibri
    [A,B]=Stab(sys,states,inputs, ...
        PointOfEquilibrium(xpred(:,n-1).'+punctDeEchilibru.States(1:9),u(:,n-1).'+punctDeEchilibru.Inputs));
    Ad=double(round(A,4));
    Bd=double(round(B,4));
    %recaluculam greutatea predictiei
    Pred=Ad*P*Ad.'+Q;
    %recalculam castig kalman
    Kn=Pred*Cd.'*inv(Cd*Pred*Cd.'+R);
    %facem estimare
    xest(:,n)=xpred(:,n)+Kn*(y(:,n)-Cd*xpred(:,n));
    %actualizam P
    P=(eye(9)-Kn*Cd)*Pred*(eye(9)-Kn*Cd).'+Kn*R*Kn.';
end
figure
for i=1:9
    xest(i,:)=xest(i,:)+punctDeEchilibru.States(i);
    subplot(9,1,i)
    plot(xest(i,:))
    hold on
    y(i,:)=y(i,:)+punctDeEchilibru.States(i);
    subplot(9,1,i)
    plot(y(i,:))
    hold on
    x(i,:)=x(i,:)+punctDeEchilibru.States(i);
    subplot(9,1,i)
    plot(x(i,:))
    legend('estimate','read','true value')
end

end