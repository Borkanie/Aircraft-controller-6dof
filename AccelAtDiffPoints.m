%pozitiole sunt relative la mine la birou
Point1=[-1.11, 0.08, -0.05];M1=sqrt(sum(Point1.^2))%X-down,y-spre mine,z-dreapta
Point2=[-0.11, 0.07, 0.88];M2=sqrt(sum(Point2.^2))%x-stanga,y-spre mine,z-down
Point3=[-0.12, 1.07, -0.30];M3=sqrt(sum(Point3.^2))%x-stanga,y-up,z-spre mine
Point4=[0.89, 0.07, -0.17];M4=sqrt(sum(Point4.^2))%x-up,y-spre mine,z stanga
Point5=[-0.10, -0.92, -0.03];M5=sqrt(sum(Point5.^2))%x-stanga,y-down,z-spre monitor
Point6=[-0.15, 0.07, -1.13];M6=sqrt(sum(Point6.^2))%xstanga,y-spre monitor, z-up
syms offX offY offZ biasX biasY biasZ
off=[offX;
    offY;
    offZ];
bias=[biasX;
    biasY;
    biasZ];
eq=[sum(([Point4].'+bias).^2)==1;
    sum(([Point5].'+bias).^2)==1;
    sum(([Point6].'+bias).^2)==1;]
res=solve(eq,[biasX biasY biasZ]);
biasX=vpa(res.biasX(1),4)
biasY=vpa(res.biasY(1),4)
biasZ=vpa(res.biasZ(1),4)
figure
plot3(Point1(1),Point1(2),Point1(3),'*')
text(Point1(1),Point1(2),Point1(3),'Point1')
hold on
grid on
plot3(Point2(1),Point2(2),Point2(3),'*')
text(Point2(1),Point2(2),Point2(3),'Point2')
hold on
plot3(Point3(1),Point3(2),Point3(3),'*')
text(Point3(1),Point3(2),Point3(3),'Point3')
hold on
plot3(Point4(1),Point4(2),Point4(3),'*')
text(Point4(1),Point4(2),Point4(3),'Point4')
hold on
plot3(Point5(1),Point5(2),Point5(3),'*')
text(Point5(1),Point5(2),Point5(3),'Point5')
hold on
plot3(Point6(1),Point6(2),Point6(3),'*')
text(Point6(1),Point6(2),Point6(3),'Point6')
[X,Y,Z]=sphere(50);
hold on
plot3(X,Y,Z)
xlabel('AccelX')
ylabel('AcelY')
zlabel('AccelZ')
%%
figure
plot3(Point1(1)+biasX,Point1(2)+biasY,Point1(3)+biasZ,'*')
text(Point1(1)+biasX,Point1(2)+biasY,Point1(3)+biasZ,'Point1')
hold on
grid on
plot3(Point2(1)+biasX,Point2(2)+biasY,Point2(3)+biasZ,'*')
text(Point2(1)+biasX,Point2(2)+biasY,Point2(3)+biasZ,'Point2')
hold on
plot3(Point3(1)+biasX,Point3(2)+biasY,Point3(3)+biasZ,'*')
text(Point3(1)+biasX,Point3(2)+biasY,Point3(3)+biasZ,'Point3')
hold on
plot3(Point4(1)+biasX,Point4(2)+biasY,Point4(3)+biasZ,'*')
text(Point4(1)+biasX,Point4(2)+biasY,Point4(3)+biasZ,'Point4')
hold on
plot3(Point5(1)+biasX,Point5(2)+biasY,Point5(3)+biasZ,'*')
text(Point5(1)+biasX,Point5(2)+biasY,Point5(3)+biasZ,'Point5')
hold on
plot3(Point6(1)+biasX,Point6(2)+biasY,Point6(3)+biasZ,'*')
text(Point6(1)+biasX,Point6(2)+biasY,Point6(3)+biasZ,'Point6')
[X,Y,Z]=sphere(50);
hold on
plot3(X,Y,Z)
xlabel('AccelX')

ylabel('AcelY')
zlabel('AccelZ')