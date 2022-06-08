states=[11.8,0,1.8,0,0,0,0,0.1,0,0,0,100];
inputs=[5.5,0,0,0,0];
Controllers=OperatingPoint(states,inputs,[-3,-2,-2]);
for uw=-0:1:1
    for vw=-0:1:1
        for ww=-0:1:1
            winds=[uw,vw,ww];
            operatingPoint=OperatingPoint([states(1:3)+winds states(4:12)],inputs,winds);
            Controllers=[Controllers operatingPoint]
        end
    end
end