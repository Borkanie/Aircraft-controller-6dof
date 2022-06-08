states=[11.8,0,1.8,0,0,0,0,0.1,0,0,0,100];
inputs=[5.5,0,0,0,0];
pointOfEquilibrium=PointOfEquilibrium(states,inputs,[-1,-1,-1]);
Ts=0.1;
[AEmd,BEmd,CEmd,KEmd,Cm]=ControlMinimal(pointOfEquilibrium);