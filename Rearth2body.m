function [R] = Rearth2body(Phi,Theta,Psi)
R=rotx(Phi)*roty(Theta)*rotz(Psi);
end