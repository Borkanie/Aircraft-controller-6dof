function  [AEmd,BEmd,CEmd,LEd]=EstimareIntrareNecunoscuta(sys,perturbations,jordanA,jordanB,states,inputs,Kd,punctDeEchilibru,Ts)
jordanE=systemGradient(sys,perturbations)
vant=[1,1,1]*0;
JA=[jordanA,jordanE(1:9,:);zeros(3,12)];
JB=[jordanB;zeros(3,5)];
AE=subs(JA,[states,inputs,perturbations.'],[punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
BE=subs(JB,[states,inputs,perturbations.'],[punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
CE=[eye(9),zeros(9,3)];
rank(obsv(AE,CE))

% estimator de stare in discret cu control pe nelienar
%perioada de esantionara
%o perioada mai mica nu ar permite timp de calcul destul de mare pe masina
%de la bordul avionului
AEmd=double(eye(12)+Ts*AE);
BEmd=double(Ts*BE);
CEmd=CE;
%verificam observabilitate
rank(obsv(AEmd,CEmd))

%observer discret
%impunem poli
ObsPoles=[-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12];
LEd=place(AEmd',CEmd',ObsPoles).';
eig(AEmd-LEd*CEmd)


end