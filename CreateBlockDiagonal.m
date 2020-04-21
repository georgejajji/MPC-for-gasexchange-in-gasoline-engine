% Created by Robin Holmbom Ph.D. Student at Linkoping University
% Creates a block diagonal matrix of A with dimension n.
% A: k x m
% B: k*n x m*n
% B = [A 0 0 ... 0; 
%      0 A 0 ... 0; 
%      0 0 A ... 0;
%      0 0 0 ... A] 
function B = CreateBlockDiagonal(A,n)
    B = kron(eye(n),A);
end
