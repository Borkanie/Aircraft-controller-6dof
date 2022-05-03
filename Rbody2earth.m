function [R] = Rbody2earth(Phi,Theta,Psi)
R=rotz(Psi).'*roty(Theta).'*rotx(Phi).';
end