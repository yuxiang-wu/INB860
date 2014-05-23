function [c , M1] = myBehaviorA3(La, d, M0)
%
% The robot is just entering an intersection
%   La :  list of the neighbors in degrees anticlockwise of the incident branches with respect to the branch the robot is on
if(~isfield(M0, 'steps'))
    M0.steps = 10;
    M0.curstep = 1;
else
    M0.curstep = M0.curstep + 1;
end

if(M0.curstep <= M0.steps)
    c = ceil(length(La)*rand());
else
    if ~isfield(M0, 'btPos')
        M0.btPos = length(M0.logbook)+1;
        c = 0;
    else
        M0.btPos = M0.btPos - 1;
        l = M0.logbook(M0.btPos, 2);
        c = mod(l - M0.logbook(M0.btPos, 1), l);
    end
    
end

if ~isfield(M0,'logbook')
    M0.logbook = [c length(La)+1];
else
    M0.logbook = [M0.logbook ;[c length(La)+1] ];
end

M1 = M0;