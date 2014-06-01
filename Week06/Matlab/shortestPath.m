function [result, path] = shortestPath(M)
%
% result: shortest path from current position on M to the start, used in A4
% path: shortest path from start to current position without deleting the zero at the beginning, used in A5
%

if(isempty(M.logbook)) % special case: empty logbook
    result = [];
    return
elseif(size(M.logbook,1) <= 2) % special case: small logbook
    tmp = M.logbook;
else
    tmp = M.logbook(1:2,:);
    for i= 3: size(M.logbook, 1)
        % preprocess tmp in consideration of special cases
        if(~isempty(tmp))
            if(tmp(end,1) == 0 && M.logbook(i,1) == 0)
                tmp(end,:)=[];
                continue
            end
        end
        
        % the core of this algorithm
        if(size(tmp,1)>=2)
            if(tmp(end-1,1)~=0 && tmp(end, 1)==0 && M.logbook(i,1)~=0) % there is a U turn at the end of the tmp
                b = tmp(end-1, 2);
                a = mod(tmp(end-1, 1) + M.logbook(i,1), b);
                % replace the last two elements in tmp with [a b]
                tmp(end-1:end,:)=[];
                tmp = [tmp; a b];
            else
                % append new element
                tmp = [tmp; M.logbook(i,:)];
            end
        else
            % append new element
            tmp = [tmp; M.logbook(i,:)];
        end
    end
end

path = tmp;

% post processing: deleting all the zero rows in tmp at the begining
while(~isempty(tmp))
    if(tmp(1:1)==0)
        tmp(1,:)=[];
    else
        break;
    end
end

% special case: empty tmp, return empty matrix directly
if(isempty(tmp))
    result=[];
    return
end

result = [];
for i = 1 : size(tmp,1)
    result = [result; tmp(i,2)-tmp(i,1) tmp(i,2)];
end

% determine whether it need to take a U turn before it backtracks
if(tmp(end,1)~=0)
    result = [result; 0 1];
else
    result(end,:)=[];
end
