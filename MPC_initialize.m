%number of states
nx=9;
%number of outputs
ny=9;

airsystem=ss(A(1:9,1:9),B(1:9,:),eye(9),zeros(9,5));
airsystem=c2d(airsystem,Ts);

Mpc=mpc(airsystem,Ts,1,1);

Mpc.Weights.OutputVariables=[1,1,1,0,0,0,1,1,1];
Mpc.Weights.ManipulatedVariables = [0.1 1 1 1 1];
Mpc.Weights.ManipulatedVariablesRate = [0.1 1 1 1 1];
Mpc.MV(1).Min=0;
Mpc.MV(1).Max=10;
Mpc.MV(2).Min=-pi/6;
Mpc.MV(2).Max=pi/6;
Mpc.MV(3).Min=-pi/6;
Mpc.MV(3).Max=pi/6;
Mpc.MV(4).Min=-pi/6;
Mpc.MV(4).Max=pi/6;
Mpc.MV(5).Min=-pi/6;
Mpc.MV(5).Max=pi/6;

initialMpc=mpcstate(Mpc);
initialMpc.Plant=pointOfEquilibrium.States(1:9);
initialMpc.LastMove=pointOfEquilibrium.Inputs;
