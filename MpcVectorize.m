% Function that vectorizes the discretized system.
% X = Fx(k)+GU
% U = [u(k),u(k+1),...,u(k+N-1)]';
% X = [x(k),x(k+1),...,x(k+N-1)]';
% F = [I, A, A^2, ..., A^(N-1)]';
% G = [0, 0, 0, ..., 0; 
%      B, 0, 0, ..., 0; 
%      AB, B, 0, ..., 0;
%      ......;
%      A^(N-2)B,...,AB,B,0];
% Q1 = diag([q1,q1,...,q1]);
% Q2 = diag([q2,q2,...,q2]);
% M = diag([c,c,...,c]);
function [F,G,M] = MpcVectorize(A,B,C,N,m,n)
    F=eye(n);
    for kk = 1:N-1
        F = [F;A^kk];
    end

    G = zeros(n*N,m*N);
    for kk = 1:N-1
        G = G + kron(diag(ones(1,N-kk),-kk),A^(kk-1)*B);
    end
    M = kron(eye(N),C);
end