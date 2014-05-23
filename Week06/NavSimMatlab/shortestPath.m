function [result, path] = shortestPath(M)
if(isempty(M.logbook))
    result = [];
    return
elseif(size(M.logbook,1) <= 2)
    tmp = M.logbook;
else
    tmp = M.logbook(1:2,:);
    for i=3:size(M.logbook,1)
        if(~isempty(tmp))
            if(tmp(end,1)==0 && M.logbook(i,1)==0)
                tmp(end,:)=[];
                continue
            end
        end
        
        if(size(tmp,1)>=2)
            if(tmp(end-1,1)~=0 && tmp(end, 1)==0 && M.logbook(i,1)~=0)
                b = tmp(end-1, 2);
                a = mod(tmp(end-1, 1) + M.logbook(i,1), b);
                tmp(end-1:end,:)=[];
                tmp = [tmp; a b];
            else
                tmp = [tmp; M.logbook(i,:)];
            end
        else
            tmp = [tmp; M.logbook(i,:)];
        end
    end
end

path = tmp;
while(~isempty(tmp))
    if(tmp(1:1)==0)
        tmp(1,:)=[];
    else
        break;
    end
end

if(isempty(tmp))
    result=[];
    return
end

result = [];
for i = 1 : size(tmp,1)
    result = [result; tmp(i,2)-tmp(i,1) tmp(i,2)];
end

if(tmp(end,1)~=0)
    result = [result; 0 1];
else
    result(end,:)=[];
end
