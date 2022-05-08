function [] = KalmanFilterForLinearSystem(Ad,Bd,Cd,Kd,x0,xest0,punctDeEchilibru)
R=eye(9)*10;
Q=eye(9);
%pedictie de precizie intiala
P=eye(9);
%numar de iterati
iter=100;
x=x0;
xest=xest0;
xpred=zeros(9,1);
for n=2:iter
    u(:,n-1)=-Kd*x(:,n-1);
    process_noise=(rand(9,1)-rand(9,1))*3;
    x(:,n)=Ad*x(:,n-1)+Bd*u(:,n-1)+process_noise;
    reading_noise=(rand(9,1)-rand(9,1))*3;
    y(:,n)=Cd*x(:,n)+reading_noise;
    [xpnou,P]=KalmanFilterNoCap(Ad, Bd, y(:,n), Cd, xpred(:,n-1), u(:,n-1), P, Q, R);
    xpred(:,n)=xpnou;
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