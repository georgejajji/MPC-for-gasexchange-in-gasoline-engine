% Generates the upcoming time vector in the MPC. 
% timeVec = [t0, t0+Ts, t0+2*Ts,...,t0+(N-1)*Ts]
% Mainly used for reference generation.
function [timeVec] = Mpc_GenerateNextTimeVec(t0,N,Ts)
    timeVec = t0*ones(N,1)+(0:1:N-1)'*Ts;
end