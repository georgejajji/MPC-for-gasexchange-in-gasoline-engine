% Function that vectorizes the discretized system.
% For x(k) = 0 then F is not needed, which is the case for linearized
% system.
% X = Fx(k)+GU+D
% U = [u(k),u(k+1),...,u(k+N-1)]';
% X = [x(k),x(k+1),...,x(k+N-1)]';
% F = [I, A, A^2, ..., A^(N-1)]';
% G = [0, 0, 0, ..., 0; 
%      B, 0, 0, ..., 0; 
%      AB, B, 0, ..., 0;
%      ......;
%      A^(N-2)B,...,AB,B,0];
% D = [0,I*K,I*K+A*K,I*K+A*K+A^2*K,...,I*K+A*K+A^2*K+...+A^(N-2)*K]';
% Q1 = diag([q1,q1,...,q1]);
% Q2 = diag([q2,q2,...,q2]);
% M = diag([c,c,...,c]);
function [G,D,M] = MpcVectorizeLinearized(A,B,C,K,N,m,n)
    D = zeros(n*N,1);
    temp = zeros(n,1);
    for kk = 1:N-1
        temp = temp + A^(kk-1)*K;
        D(n*kk+1:n*(kk+1)) = temp;
    end

    G = zeros(n*N,m*N);
    for kk = 1:N-1
        G = G + kron(diag(ones(1,N-kk),-kk),A^(kk-1)*B);
    end
    M = kron(eye(N),C);
end