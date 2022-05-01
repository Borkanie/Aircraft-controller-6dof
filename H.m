function [H] = H(Phi,Theta)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
H=[1,tan(Theta)*sin(Phi),tan(Theta)*cos(Phi);
    0,cos(Phi),-sin(Phi);
    0,sin(Phi)/cos(Theta),cos(Phi)/cos(Theta)];
end