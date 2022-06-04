function [] = KalmanFilterForExtendedLinearSystem(Ad,Bd,Cd,Kd,punctDeEchilibru)
R=eye(9)/1;
Q=eye(12)*1;
Q(10:12,10:12)=5*Q(10:12,10:12);
equilibriumpoints=[punctDeEchilibru.States(1:9).';punctDeEchilibru.States(13:15).'];
%pedictie de precizie intiala
P=eye(12)/100000;
%numar de iterati
iter=500;
Ts=0.1;
x=[punctDeEchilibru.States(1:9).';[1,1,1].'];
xest=zeros(12,1);
y=x(1:9);
for n=2:iter
    u(:,n-1)=-Kd*xest(:,n-1)+punctDeEchilibru.Inputs.';
    if n>30
        punctDeEchilibru.States(13:15)=[1,2,1];
    else
        punctDeEchilibru.States(13:15)=[1,1,1];
    end
    dxdt=aircraftSystem([x(1:12,n-1)],u(:,n-1),punctDeEchilibru.States(13:15).');
    x(:,n)=x(:,n-1)+[dxdt(1:9)*Ts;zeros(3,1)];
    x(10:12,n)=punctDeEchilibru.States(13:15).';
    reading_noise=[(rand(3,1)-rand(3,1))*.3;(rand(6,1)-rand(6,1))*.1];
    y(:,n)=Cd*x(:,n)+[reading_noise];
        xpred=Ad*xest(:,n-1)...
            +Bd*(u(:,n-1)-punctDeEchilibru.Inputs.');
        Pred=Ad*P*Ad.'+Q;
        Kn=Pred*Cd.'/(Cd*Pred*Cd.'+R);
        xest(:,n)=(xpred+Kn*((y(:,n-1)-punctDeEchilibru.States(1:9).')- ...
            Cd*xpred));
        P=(eye(12)-Kn*Cd)*Pred*(eye(12)-Kn*Cd).'+Kn*R*Kn.';

end
punctDeEchilibru.States=[punctDeEchilibru.States(1:9).';punctDeEchilibru.States(13:15).'];

f1=figure(1);
clf(f1)
for i=1:12    
    subplot(12,1,i)
    plot(xest(i,:)+equilibriumpoints(i))
    hold on   
    subplot(12,1,i)
    plot(x(i,:))
    legend('estimate','true value')
end
f2=figure(2);
clf(f2)
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
f3=figure(3);
clf(f3)
for i=1:9    
    subplot(9,1,i)
    plot(y(i,:))
    hold on   
    subplot(9,1,i)
    plot(x(i,:))
    legend('read','true value')
end
end