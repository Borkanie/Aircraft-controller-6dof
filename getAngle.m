function [rad] = getAngle(angle)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
rad=rem(angle,2*pi);
if rad>pi
    rad=rad-2*pi;
else
    if rad<-pi
        rad=rad+2*pi;
    end
end
end