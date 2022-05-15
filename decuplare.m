function  [AEmd,BEmd,CEmd,LEd]=decuplare(sys,perturbations,jordanA,jordanB,states,inputs,Kd,punctDeEchilibru,Ts)
jordanE=systemGradient(sys,perturbations)
vant=[1,1,1]*0;
E=subs(jordanE,[states,inputs,perturbations.'],[punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
Adecup=subs(jordanA,[states,inputs,perturbations.'],[punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
Bdecup=subs(jordanB,[states,inputs,perturbations.'],[punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
E=double(E(1:9,:))
Adecup=double(Adecup(1:9,1:9));
Bdecup=double(Bdecup(1:9,:));
CE=[eye(9)];

% estimator de stare in discret cu control pe nelienar
%perioada de esantionara
Ts=.1;
%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
AEmd=double(eye(9)+Ts*Adecup);
BEmd=double(Ts*Bdecup);
CEmd=CE;
%verificam observabilitate
rank(obsv(AEmd,CEmd))

%observer discret
%impunem poli
H=E*pinv(CE*E)
T=eye(9)-H*CE

ObsPoles=[-1,-2,-3,-4,-5,-6,-7,-8,-9];
K1=place([Adecup-H*CE*Adecup].',CE.',ObsPoles).';
eig(Adecup-H*CE*Adecup-K1*CE)

F=T*Adecup-K1*CE
K2=F*H


end