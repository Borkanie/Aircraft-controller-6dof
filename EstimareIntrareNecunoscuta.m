function  [E,AEmd,BEmd,CEmd,LEd,vant]=EstimareIntrareNecunoscuta(sys,perturbations,jordanA,jordanB,states,inputs,Kd,punctDeEchilibru,Ts)
jordanE=systemGradient(sys,perturbations);
vant=[.01,.01,.01];
punctDeEchilibru.States(1:3)=punctDeEchilibru.States(1:3)-vant;


E=double(subs(jordanE,[states,inputs,perturbations.'], ...
    [punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]));
JA=[jordanA,jordanE(1:9,:);zeros(3,12)];
JB=[jordanB;zeros(3,5)];
AE=subs(JA,[states,inputs,perturbations.'], ...
    [punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
BE=subs(JB,[states,inputs,perturbations.'], ...
    [punctDeEchilibru.States(1:9),punctDeEchilibru.Inputs,vant]);
CE=[eye(9),[eye(3,3);zeros(6,3)]];
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
ObsPoles=[-5.1,-4.2,-3.3,-4.7,-5,-1.6,-3,-4,-2,-5.1,-4.2,-3.3];
LEd=place(AEmd',CEmd',ObsPoles).'
eig(AEmd-LEd*CEmd);

rank(ctrb(AEmd,BEmd));

end