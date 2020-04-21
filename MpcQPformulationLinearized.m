% Created by Robin Holmbom Ph.D. Student at Linkoping University
% Translates problem to a QP formulation with reference following.
% This is used for a linearized system.
function [H,g] = MpcQPformulationLinearized(G,M,D)
    H = @(Q1,Q2) transpose(G)*transpose(M)*Q1*M*G+Q2;
    g = @(x0,Q1,Q2,U0,R,one_vector) transpose(G)*transpose(M)*Q1*(M*(x0+D)-R) + Q2*U0; 
end








%EXTRA
% function [H,g] = MpcQPformulationLinearized(G,M,D,Omega,delta)
%     H = @(Q1,Q2) transpose(G)*transpose(M)*Q1*M*G+transpose(Omega)*Q2*Omega;
%     g = @(x0,Q1,Q2,U0,R,delta) transpose(G)*transpose(M)*Q1*(M*(x0+D)-R) + Q2*U0 -transpose(Omega)*Q2*delta; 
% end
