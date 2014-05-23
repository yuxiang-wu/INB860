function [c , M1] = myBehaviorA4(La, d, M0)
%
% The robot is just entering an intersection
%   La :  Last of the neighbors in degrees anticlockwise of the incident branches with respect to the branch the robot is on

if(~isfield(M0, 'steps'))
    M0.steps = 8;
    M0.curstep = 1;
else
    M0.curstep = M0.curstep + 1;
end

if(M0.curstep <= M0.steps)
    c = ceil(length(La)*rand());


else
    if ~isfield(M0, 'btPos')
        [M0.shortestPath, ~] = shortestPath(M0);
        M0.btPos = size(M0.shortestPath,1);
    end
    
    if(M0.btPos > 0)
        c = M0.shortestPath(M0.btPos, 1);
        M0.btPos = M0.btPos - 1;
    else
        c = -1;
    end
    
end

if ~isfield(M0,'logbook')
    M0.logbook = [c length(La)+1];
else
    M0.logbook = [M0.logbook ;[c length(La)+1] ];
end

M1 = M0;
