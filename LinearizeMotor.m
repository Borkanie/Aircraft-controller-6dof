listeInputs=[0,50,100,150,200,250,300,350];
listeGravity=[4820,4753,4610,4460,4320,4200,4050,3900];
DefaultValue=4820;
plot(listeInputs,(4820-listeGravity)*9.81/1000,'*')
xlabel('PWM')
ylabel('Newtons')
hold on
Linear=polyfit(listeInputs,(4820-listeGravity)*9.81/1000,1);
Y1=polyval(Linear,listeInputs);
plot(listeInputs,Y1)