function [A,B] = Stab(system,states,inputs,pointOfEquilibrium)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
jordanA=systemGradient(system,states);
jordanB=systemGradient(system,inputs);
A=subs(jordanA,[states,inputs],[pointOfEquilibrium.States,pointOfEquilibrium.Inputs]);
B=subs(jordanB,[states,inputs],[pointOfEquilibrium.States,pointOfEquilibrium.Inputs]);
end