function [Cl] = GetWingLift(alpha)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if (alpha<20) && (alpha>-10)
    Cl=0.11*alpha*180/pi;
    return;
end
if alpha<-10
    Cl=0;
    return;
end
Cl=0.11*20*180/pi;

end