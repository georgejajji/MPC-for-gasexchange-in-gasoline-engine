% Created by Robin Holmbom Ph.D. Student
% Translates problem to a QP formulation with reference following.
% This is used for a linearized system.
function [H,g] = MpcQPformulation(F,G,M)
    H = @(Q1,Q2) transpose(G)*transpose(M)*Q1*M*G+Q2;
    g = @(x0,Q1,R) transpose(G)*transpose(M)*Q1*(M*F*x0-R); 
end

