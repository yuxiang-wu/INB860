function [c , M1] = TheseusCrawler(Li, M0, isBacktrack)
%
% The robot is just entering an intersection
%   Li :  list of the neighbors in degrees anticlockwise of the incident branches with respect to the branch the robot is on

if(~isBacktrack)
    c = ceil(length(Li)*rand());

    if ~isfield(M0,'logbook')
        M0.logbook = [c length(Li)+1];
    else
        M0.logbook = [M0.logbook ;[c length(Li)+1] ];
    end
        
    M1 = M0;
else
    if ~isfield(M0, 'btPos')
        M0.btPos = []

end
