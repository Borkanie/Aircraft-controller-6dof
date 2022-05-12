function [] = KalmanFilterExtendedForLinearSystem(jordanA,jordanB,states,inputs,Kd,xest0,punctDeEchilibru,Ad,Bd,Ts)
punctDeEchilibru=PointOfEquilibrium(punctDeEchilibru.States(1:9).',punctDeEchilibru.Inputs.');
R=eye(9)*10;
Q=eye(9);
%rpedictie de precizie intiala
P=eye(9);
%numar de iterati
iter=10;
Cd=eye(9);
x=punctDeEchilibru.States(1:9);
xest=xest0;
for n=2:iter
    u(:,n-1)=-Kd*(x(:,n-1)-punctDeEchilibru.States);
    process_noise=(rand(9,1)-rand(9,1))*0;
    dxdt=aircraftSystem([x(:,n-1);0;0;0],u(:,n-1)+punctDeEchilibru.Inputs,process_noise(1:3));
    x(:,n)=x(:,n-1)+dxdt(1:9)*Ts;
    reading_noise=(rand(9,1)-rand(9,1))*0;
    y(:,n)=Cd*x(:,n)+reading_noise;
    xpred=Ad*(xest(:,n-1))+Bd*(u(:,n-1));
    [xpred,P,Ad,Bd] = KalmanFilterExtendedCox(jordanA,jordanB, states, ...
        inputs,xest(:,n-1), xpred, u(:,n-1), P, Q, Cd, R, y(:,n-1),Ts,punctDeEchilibru);
    punctDeEchilibru=PointOfEquilibrium(x(:,n-1),u(:,n-1));
    xest(:,n)=xpred;
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