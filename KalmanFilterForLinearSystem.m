function [] = KalmanFilterForLinearSystem(Ad,Bd,Cd,Kd,x0,punctDeEchilibru)
R=eye(9)*1000;
Q=eye(9);
%pedictie de precizie intiala
P=eye(9);
%numar de iterati
iter=150;
Ts=0.1;
x=x0;
xest=zeros(9,1);
for n=2:iter
    u(:,n-1)=-Kd*(xest(:,n-1))+punctDeEchilibru.Inputs.';
    process_noise=(rand(9,1)-rand(9,1))*0.5;
    dxdt=aircraftSystem([x(:,n-1);0;0;0],u(:,n-1),process_noise(1:3));
    x(:,n)=x(:,n-1)+dxdt(1:9)*Ts;
    reading_noise=(rand(9,1)-rand(9,1))*.2;
    y(:,n)=Cd*x(:,n)+reading_noise;
    [xpred,P]=KalmanFilterNoCap(Ad, Bd, (y(:,n)-punctDeEchilibru.States(1:9).'), ...
        Cd, xest(:,n-1), u(:,n-1)-punctDeEchilibru.Inputs.', P, Q, R);
    xest(:,n)=xpred;
end
figure
for i=1:9
    xest(i,:)=xest(i,:)+punctDeEchilibru.States(i);
    subplot(9,1,i)
    plot(xest(i,:))
    hold on
    subplot(9,1,i)
    plot(y(i,:))
    hold on
    subplot(9,1,i)
    plot(x(i,:))
    legend('estimate','read','true value')
end
figure
    subplot(5,1,1)
    plot(u(1,:))
    subplot(5,1,2)
    plot(u(2,:))
    subplot(5,1,3)
    plot(u(3,:))
    subplot(5,1,4)
    plot(u(4,:))
    subplot(5,1,5)
    plot(u(5,:))

end