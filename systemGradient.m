function [jordan] = systemGradient(system,states)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
jordan=gradient(system(1),states).';
for i=2:length(system)
    jordan=[jordan;gradient(system(i),states).'];
end
end 