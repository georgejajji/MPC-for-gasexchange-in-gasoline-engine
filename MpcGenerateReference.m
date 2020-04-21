% Generate reference following, with or without preview. 
% preview = ~0: use r_func to get the upcoming reference
% preview = 0: use the same upcoming reference as of now
function [R] = MpcGenerateReference(r_func,t,preview)
    if preview
        R = r_func(t);
    else
        R = r_func(t(1))*ones(size(t));
    end
end