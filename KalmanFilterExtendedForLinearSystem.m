function [] = KalmanFilterExtendedForLinearSystem(jordanA,jordanB,states,inputs,Kd,punctDeEchilibru,Ad,Bd,Ts)
punctDeEchilibru=PointOfEquilibrium(punctDeEchilibru.States(1:9).',punctDeEchilibru.Inputs.');
po=punctDeEchilibru;
R=eye(9);
R(1:3,1:3)=R(1:3,1:3)*100;
Q=eye(9);
Q(4:9,4:9)=Q(4:9,4:9)/10;
%pedictie de precizie intiala
P=eye(9)/100000;
%numar de iterati
iter=10;
Cd=eye(9);
x=punctDeEchilibru.States(1:9);
xest=zeros(9,1);
y=x;
for n=2:iter
    u(:,n-1)=-Kd*(x(:,n-1)-po.States)+punctDeEchilibru.Inputs;
    process_noise=(rand(3,1)-rand(3,1))*0;
    dxdt=aircraftSystem([x(:,n-1);0;0;0],u(:,n-1),process_noise);
    x(:,n)=x(:,n-1)+dxdt(1:9)*Ts;
    reading_noise=[(rand(3,1)-rand(3,1))*.5;
        (rand(3,1)-rand(3,1))*.1;
        (rand(3,1)-rand(3,1))*.1;];
    y(:,n)=Cd*x(:,n)+reading_noise;
    
    xpred=Ad*(xest(:,n-1))+Bd*(u(:,n-1)-punctDeEchilibru.Inputs);
    [xpred,P,Ad,Bd] = KalmanFilterExtendedCox(jordanA,jordanB, states, ...
        inputs,xest(:,n-1), xpred, u(:,n-1), P, Q, Cd, R, y(:,n),Ts,punctDeEchilibru);
     xest(:,n)=xpred;
    punctDeEchilibru=PointOfEquilibrium(xest(:,n-1)+punctDeEchilibru.States,u(:,n-1));
   
    
end
figure
for i=1:9
    xest(i,:)=xest(i,:)+po.States(i);
    subplot(9,1,i)
    plot(xest(i,:))
    hold on
    y(i,:)=y(i,:);
    subplot(9,1,i)
    plot(y(i,:))
    hold on
    x(i,:)=x(i,:);
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