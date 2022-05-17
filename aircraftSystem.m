function dxdt = aircraftSystem(x,u,d)
obj=aircraft();
g=9.81;
%take inputs from vector
T=u(1);
DeltaSt=getAngle(u(2));
DeltaRud=getAngle(u(3));
DeltaAlL=getAngle(u(4));
DeltaAlR=getAngle(u(5));
%take states from vector
u=x(1)-d(1); 
v=x(2)-d(2);
w=x(3)-d(3);
p=x(4);
q=x(5);
r=x(6);
Phi=(x(7));
Theta=(x(8));
Psi=(x(9));
px=x(10);
py=x(11);
pz=x(12);
%vant in earth system
R=Rearth2body(Phi,Theta,Psi);
d=R*d;


alpha=atan2(w,u);
beta=atan2(v,u);
Va2=(u)^2+(v)^2+(w)^2;
Cdw=0.0115*alpha*180/pi+1;
Cdfus=0.075*alpha*180/pi+1;
CyFus=0.075*beta*180/pi;
Cyrud=0.075*(beta+DeltaRud)*180/pi;
Clw=0.11*alpha*180/pi;
ClSt=0.11*(alpha+DeltaSt)*180/pi;
ClwL=0.11*(DeltaAlL+alpha)*180/pi;
ClwR=0.11*(DeltaAlR+alpha)*180/pi;
G=[0;0;obj.m*g];
Fa=[-(Cdw*obj.Af_wing+Cdfus*obj.Af_fuselage)*obj.ro*Va2/2;
-(CyFus*obj.Alat+Cyrud*obj.Arudder)*obj.ro*Va2/2;
-(Clw*obj.Aw+ClSt*obj.Ast+ClwL*obj.Aaileron/2+ClwR*obj.Aaileron/2)*obj.ro*Va2/2];
l_fus=0.01;
Ft=[T;0;0];
% fara vant alpha=theta
% beta=psi
F=Ft+Rearth2body(0,Theta,Psi)*Fa+R*G;

L_bar=obj.l3*obj.Aaileron*(ClwL-ClwR)*obj.ro*Va2/2;
M=(obj.l1*Clw*obj.Aw+ClSt*obj.l2*obj.Ast)*obj.ro*Va2/2;
N=(obj.l2*Cyrud*obj.Arudder+CyFus*l_fus*obj.Alat)*obj.ro*Va2/2;

Moments=[L_bar;M;N];
dxdt=[F/obj.m-cross([p;q;r],[u;v;w]);
    obj.Iinv*(Moments-cross([p;q;r],obj.I*[p;q;r]));
    H(Phi,Theta)*[p;q;r];
    Rbody2earth(-Phi,-Theta,-Psi)*[u;v;w]];
end