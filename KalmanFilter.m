%Kalman exercise
%vrem sa vedem covarianta la 2 echipe
teamA=[1.89,2.1,1.75,1.98,1.85,1.914];
teamB=[1.94,1.9,1.97,1.89,1.87,1.914];
%calculam mediile
ma=mean(teamA)
mb=mean(teamB)
%vedem cat sunt de impartiti
SpreadA=teamA-ma
SpreadB=teamB-mb
%ca sa scapamd e negativa facem MSE
SpreadA=SpreadA.^2
SpreadB=SpreadB.^2
%putem calcula variatia ca suma patratelor distantelor
%N-1 se numeste factoruld e corectie Bessel
VariatiaA=1/(length(SpreadA)-1)*sum(SpreadA)
VariatiaB=1/(length(SpreadB)-1)*sum(SpreadB)
%Deviatia standard este radical din variate
DstdA=sqrt(VariatiaA)
DstdB=sqrt(VariatiaB)
%% bara de aur exemplu
x=1000;
%pentru ca greutatea barii nuse se schimba systemul dinamic devine
%x(k+1)=x(k)
%facem o predictie
x=[x x];
%facem 1000 de citiri
iter=10;%g
%precizia cantului
prec=100;%g
for n=3:iter
    %facem estimare
    x(n)=x(n-1)+1/n*(1000+(rand(1)-1/2)*prec-x(n-1));
    %facem predictie
    x(n)=x(n);
end
vpa(x(length(x)-1),4)
%% exemplu alpha beta
%interval track-track
Dt=5;
%we start at n-1
x=30000;
x_dot=40;
%folosim extrapolarea starilor pentru a prexize xn,n-1
x=x+Dt*x_dot;
x_dot=x_dot;
%avem citirea zn
zn=30110;
eroar=zn-x
%CAZ I
%debiatia standard a radarului
Drad=20;
%cum eroare>Drad presupunem c avionul si-a schimbat viteza
%beta depends on the precision of the aircraft
beta=0.9;
x_dot=x_dot+beta*eroar/Dt
%CAZ II
Drad=150;
%avem eroare<Drad punem un beta mia mic
beta=0.1;
%resetam
x_dot=40;
x_dot=x_dot+beta*eroar/Dt




%interval track-track
Dt=5;
%we start at n-1
x=30000;
x_dot=40;
%folosim extrapolarea starilor pentru a prexize xn,n-1
x=[x x+Dt*x_dot];
x_dot=[x_dot x_dot];
alpha=0.2;
beta=0.1;
iter=7;
z=x(2)+(rand(1)-1/2)*100;
for n=2:iter
    %facem estimare
    x(n)=x(n)+alpha*(z-x(n));
    x_dot(n)=x_dot(n)+beta*(z-x(n))/Dt;
   
    %facem predictie
    x(n+1)=x(n)+Dt*x_dot(n);
    x_dot(n+1)=x_dot(n);
    %citim cu zgomot
    z=x(n+1)+(rand(1)-1/2)*300;
end
vpa(x(length(x)-1),4)
vpa(x_dot(length(x_dot)-1),4)
%% exemplu avion fara control sistem multidimensional

%timpul track-to-track al radarului(timp de citire)
Dt=5;
A=eye(9);
for i=1:6
    A(i,i+3)=Dt;
    if i<=3
        A(i,i+6)=1/2*Dt^2;
    end
end
%% exemplu avion cu control sistem multidimensional

%timpul track-to-track al radarului(timp de citire)
Dt=5;
A=eye(6);
for i=1:3
    A(i,i+3)=Dt;
end
B=zeros(6,3);
for i=1:3
    B(i,i)=1/2*Dt^2;
    B(i+3,i)=Dt;
end
