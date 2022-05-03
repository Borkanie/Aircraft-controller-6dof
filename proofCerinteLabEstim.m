%kalman proof pt lab
clear
clc
newSys
x0=zeros(9,1);
xest0=zeros(9,1);
%%
%kalman filter
KalmanFilterForLinearSystem(Amd,Bmd,Cmd,Kmd,x0,xest0,pointOfEquilibrium)

KalmanFilterExtendedForLinearSystem(sys(1:9),states(1:9),inputs,Kmd,x0,xest0,pointOfEquilibrium)
%%
%decuplare
clear 
clc
A=[ 0,1,0;
    -2,-3,0;
    0,0,-2];
B=[0,0;
   4,0;
   0,1];
E=[ 2;
    0;
    .5];
C=[1,0,0;
    0,0,1];
rank(E);
rank(C*E);

H=E*pinv(C*E);
T=eye(3)-H*C;

rank(obsv(T*A,C));

K1=[1.97,0.32;
    -2.04,-.02;
    -.16,-.86];
K2=[-1.82,-.45;
    .04,.01;
    -.08,-.02];
F=T*A-K1*C;

%initial state
x=[.5;.1;-.3];
%iunitial guess
xest=[.6;.9;-.1];
y=C*x;
z=xest-H*y;
Kdecup=lqr(A,B,eye(3),eye(2))/100;
for n=2:10
    u(:,n-1)=-Kdecup*x(:,n-1);
    process_noise=(rand(1)-rand(1))*.1;
    x(:,n)=A*x(:,n-1)+B*u(:,n-1)+E*process_noise;
    reading_noise=0;
    y(:,n)=C*x(:,n)+reading_noise;
    z(:,n)=F*z(:,n-1)+T*B*u(:,n-1)+(K1+K2)*y(:,n);   
    xest(:,n)=z(:,n)+H*y(:,n);
end
figure
for i=1:2
    xest(i,:)=xest(i,:);
    subplot(2,1,i)
    plot(xest(i,:))
    hold on
    y(i,:)=y(i,:);
    subplot(2,1,i)
    plot(y(i,:))
    hold on
    x(i,:)=x(i,:);
    subplot(2,1,i)
    plot(x(i,:))
    legend('estimate','read','true value')
end
