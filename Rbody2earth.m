function [R] = Rbody2earth(Phi,Theta,Psi)
R=rotx(Phi)*roty(Theta)*rotz(Psi);
end