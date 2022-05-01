function [A,B,C,D,U,Y,X,DX] = updateSystem(x,u)
g=9.81;
X=x(1:9);
U=u;

%take inputs from vector
DeltaSt=u(2);
DeltaRud=u(3);
DeltaAlL=u(4);
DeltaAlR=u(5);
%take states from vector
u=x(1); 
v=x(2);
w=x(3);
p=x(4);
q=x(5);
r=x(6);
Phi=x(7);
Theta=x(8);
Psi=x(9);



Ac=[(41540861029405619189*sin(Theta)*(imag(u)+real(w))*(u^2+v^2+w^2))/(23058430092136939520*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2))-(100*u*cos(Theta)*sin(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63-(100*u*cos(Psi)*cos(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(100*u*sin(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63+(47691*cos(Psi)*cos(Theta)*(imag(u)+real(w))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2))+(4011*cos(Theta)*sin(Psi)*(imag(u)+real(v))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),r-(100*v*sin(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63-(100*v*cos(Theta)*sin(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63-(100*v*cos(Psi)*cos(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63+(4011*cos(Theta)*sin(Psi)*(imag(v)-real(u))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(41540861029405619189*sin(Theta)*(imag(w)-real(u))*(u^2+v^2+w^2))/(23058430092136939520*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2))-(100*w*sin(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63-(100*w*cos(Theta)*sin(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63-(100*w*cos(Psi)*cos(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-q+(47691*cos(Psi)*cos(Theta)*(imag(w)-real(u))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),0,-w,v,0,(981*cos(Theta))/100-(50*cos(Theta)*(u^2+v^2+w^2)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63+(50*cos(Psi)*sin(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000)*(u^2+v^2+w^2))/63+(50*sin(Psi)*sin(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi))*(u^2+v^2+w^2))/63,(50*cos(Theta)*sin(Psi)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000)*(u^2+v^2+w^2))/63-(50*cos(Psi)*cos(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi))*(u^2+v^2+w^2))/63,0,0,0;
(4011*cos(Psi)*(imag(u)+real(v))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2))+(100*u*sin(Psi)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(100*u*cos(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63-r-(47691*sin(Psi)*(imag(u)+real(w))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),(100*v*sin(Psi)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(100*v*cos(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63+(4011*cos(Psi)*(imag(v)-real(u))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),p+(100*w*sin(Psi)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(100*w*cos(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63-(47691*sin(Psi)*(imag(w)-real(u))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),w,0,-u,-(981*cos(Phi)*cos(Theta))/100,(981*sin(Phi)*sin(Theta))/100,(50*cos(Psi)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000)*(u^2+v^2+w^2))/63+(50*sin(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi))*(u^2+v^2+w^2))/63,0,0,0;
q-(100*u*cos(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63+(100*u*sin(Psi)*sin(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63+(100*u*cos(Psi)*sin(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63+(41540861029405619189*cos(Theta)*(imag(u)+real(w))*(u^2+v^2+w^2))/(23058430092136939520*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2))-(4011*sin(Psi)*sin(Theta)*(imag(u)+real(v))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2))-(47691*cos(Psi)*sin(Theta)*(imag(u)+real(w))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),-p-(100*v*cos(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63+(100*v*sin(Psi)*sin(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63+(100*v*cos(Psi)*sin(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(4011*sin(Psi)*sin(Theta)*(imag(v)-real(u))*(u^2+v^2+w^2))/(12800*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(41540861029405619189*cos(Theta)*(imag(w)-real(u))*(u^2+v^2+w^2))/(23058430092136939520*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2))+(100*w*sin(Psi)*sin(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi)))/63+(100*w*cos(Psi)*sin(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000))/63-(100*w*cos(Theta)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63-(47691*cos(Psi)*sin(Theta)*(imag(w)-real(u))*(u^2+v^2+w^2))/(320000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),-v,u,0,-(981*cos(Theta)*sin(Phi))/100,-(981*cos(Phi)*sin(Theta))/100+(50*sin(Theta)*(u^2+v^2+w^2)*((101871*atan2(w,u))/(50000*pi)+(343867725467949669*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(46116860184273879040*pi)+(343*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(160000*pi)+(343*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(160000*pi)))/63+(50*cos(Psi)*cos(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000)*(u^2+v^2+w^2))/63+(50*cos(Theta)*sin(Psi)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi))*(u^2+v^2+w^2))/63,-(50*sin(Psi)*sin(Theta)*((3004533*atan2(w,u))/(16000000*pi)+11809/160000)*(u^2+v^2+w^2))/63+(50*cos(Psi)*sin(Theta)*((14553*atan2(v,u))/(40000*pi)+(147*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(64000*pi))*(u^2+v^2+w^2))/63,0,0,0;
(6663993680464069*u*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/1125899906842624+(6000*u*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231+(472311*(imag(u)+real(v))*(u^2+v^2+w^2))/(743392000*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(6663993680464069*v*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/1125899906842624+(6000*v*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231+(472311*(imag(v)-real(u))*(u^2+v^2+w^2))/(743392000*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(6663993680464069*w*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/1125899906842624+(6000*w*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231,(219550481834311640817*q)/3269472591982624768000,(219550481834311640817*p)/3269472591982624768000-(1409514093376280868051*r)/2615578073586099814400,-(1409514093376280868051*q)/2615578073586099814400,0,0,0,0,0,0;
(50*u*((101871*atan2(w,u))/(2500000*pi)-(23726873057288527161*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(11529215046068469760000*pi)))/23-(622481112949422573*(imag(u)+real(w))*(u^2+v^2+w^2))/(1325859730297874022400000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),(50*v*((101871*atan2(w,u))/(2500000*pi)-(23726873057288527161*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(11529215046068469760000*pi)))/23,(50*w*((101871*atan2(w,u))/(2500000*pi)-(23726873057288527161*((99*DeltaSt)/5+(99*atan2(w,u))/5))/(11529215046068469760000*pi)))/23-(622481112949422573*(imag(w)-real(u))*(u^2+v^2+w^2))/(1325859730297874022400000*pi*((imag(u)+real(w))^2+(imag(w)-real(u))^2)),(19*r)/23-(12*p)/115,0,(19*p)/23+(12*r)/115,0,0,0,0,0,0;
(6000*u*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/23231+(42500*u*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231+(2676429*(imag(u)+real(v))*(u^2+v^2+w^2))/(594713600*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(6000*v*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/23231+(42500*v*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231+(2676429*(imag(v)-real(u))*(u^2+v^2+w^2))/(594713600*pi*((imag(u)+real(v))^2+(imag(v)-real(u))^2)),(6000*w*((379633993036942637*((99*DeltaAlL)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)-(379633993036942637*((99*DeltaAlR)/5+(99*atan2(w,u))/5))/(2951479051793528258560*pi)))/23231+(42500*w*((14553*atan2(v,u))/(4000000*pi)-(10143*((27*DeltaRud)/2+(27*atan2(v,u))/2))/(16000000*pi)))/23231,-(937*q)/1787,-(937*p)/1787-(120*r)/1787,-(120*q)/1787,0,0,0,0,0,0;
0,0,0,1,sin(Phi)*tan(Theta),cos(Phi)*tan(Theta),q*cos(Phi)*tan(Theta)-r*sin(Phi)*tan(Theta),r*cos(Phi)*(tan(Theta)^2+1)+q*sin(Phi)*(tan(Theta)^2+1),0,0,0,0;
0,0,0,0,cos(Phi),-sin(Phi),-r*cos(Phi)-q*sin(Phi),0,0,0,0,0;
0,0,0,0,sin(Phi)/cos(Theta),cos(Phi)/cos(Theta),(q*cos(Phi))/cos(Theta)-(r*sin(Phi))/cos(Theta),(r*cos(Phi)*sin(Theta))/cos(Theta)^2+(q*sin(Phi)*sin(Theta))/cos(Theta)^2,0,0,0,0];
%{
cos(Psi)*cos(Theta),-cos(Theta)*sin(Psi),sin(Theta),0,0,0,0,w*cos(Theta)-u*cos(Psi)*sin(Theta)+v*sin(Psi)*sin(Theta),-v*cos(Psi)*cos(Theta)-u*cos(Theta)*sin(Psi),0,0,0;
cos(Phi)*sin(Psi)+cos(Psi)*sin(Phi)*sin(Theta),cos(Phi)*cos(Psi)-sin(Phi)*sin(Psi)*sin(Theta),-cos(Theta)*sin(Phi),0,0,0,-u*(sin(Phi)*sin(Psi)-cos(Phi)*cos(Psi)*sin(Theta))-v*(cos(Psi)*sin(Phi)+cos(Phi)*sin(Psi)*sin(Theta))-w*cos(Phi)*cos(Theta),w*sin(Phi)*sin(Theta)+u*cos(Psi)*cos(Theta)*sin(Phi)-v*cos(Theta)*sin(Phi)*sin(Psi),u*(cos(Phi)*cos(Psi)-sin(Phi)*sin(Psi)*sin(Theta))-v*(cos(Phi)*sin(Psi)+cos(Psi)*sin(Phi)*sin(Theta)),0,0,0;
sin(Phi)*sin(Psi)-cos(Phi)*cos(Psi)*sin(Theta),cos(Psi)*sin(Phi)+cos(Phi)*sin(Psi)*sin(Theta),cos(Phi)*cos(Theta),0,0,0,u*(cos(Phi)*sin(Psi)+cos(Psi)*sin(Phi)*sin(Theta))+v*(cos(Phi)*cos(Psi)-sin(Phi)*sin(Psi)*sin(Theta))-w*cos(Theta)*sin(Phi),v*cos(Phi)*cos(Theta)*sin(Psi)-w*cos(Phi)*sin(Theta)-u*cos(Phi)*cos(Psi)*cos(Theta),u*(cos(Psi)*sin(Phi)+cos(Phi)*sin(Psi)*sin(Theta))-v*(sin(Phi)*sin(Psi)-cos(Phi)*cos(Psi)*sin(Theta)),0,0,0;]

%}

Bc=[100/63,-(540363568592492337*sin(Theta)*(u^2+v^2+w^2))/(4611686018427387904*pi),-(63*cos(Theta)*sin(Psi)*(u^2+v^2+w^2))/(2560*pi),-(539*sin(Theta)*(u^2+v^2+w^2))/(16000*pi),-(539*sin(Theta)*(u^2+v^2+w^2))/(16000*pi);
0,0,-(63*cos(Psi)*(u^2+v^2+w^2))/(2560*pi),0,0;
0,-(540363568592492337*cos(Theta)*(u^2+v^2+w^2))/(4611686018427387904*pi),(63*sin(Psi)*sin(Theta)*(u^2+v^2+w^2))/(2560*pi),-(539*cos(Theta)*(u^2+v^2+w^2))/(16000*pi),-(539*cos(Theta)*(u^2+v^2+w^2))/(16000*pi);
0,0,-(821583*(u^2+v^2+w^2))/(743392000*pi),(250457974518265084593514314168385347*(u^2+v^2+w^2))/(33230699894622896822595176507008614400*pi),-(250457974518265084593514314168385347*(u^2+v^2+w^2))/(33230699894622896822595176507008614400*pi);
0,-(102128714463981051693*(u^2+v^2+w^2))/(2305843009213693952000*pi),0,0,0;
0,0,-(4655637*(u^2+v^2+w^2))/(594713600*pi),(563756479659859815945*(u^2+v^2+w^2))/(1714145246305386374365184*pi),-(563756479659859815945*(u^2+v^2+w^2))/(1714145246305386374365184*pi);
0,0,0,0,0;
0,0,0,0,0;
0,0,0,0,0;];
%{
0,0,0,0;
0,0,0,0;
0,0,0,0;];

Cc=eye(12);
Dc=zeros(12,4);
%}
Cc=eye(9);
Dc=zeros(9,4);
%perioada de esantionare
Ts=.01;

% Generate discrete-time model
nx = size(Ac,1);
nu = size(Bc,2);
M = expm([[Ac Bc]*Ts; zeros(nu,nx+nu)]);
A = M(1:nx,1:nx);
B = M(1:nx,nx+1:nx+nu);
C = Cc;
D = Dc;

% Nominal conditions for discrete-time plant
Y = C*X + D*U;
DX = A*X+B*U-X;

end