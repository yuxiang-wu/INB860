function [c , M1] = myBehaviorA5(La, d, M0)
%
% The robot is just entering an intersection
%   La :  Last of the neighbors in degrees anticlockwise of the incident branches with respect to the branch the robot is on

if ~isfield(M0, 'stack')
    M0.stack = 1; % stack for DFS
    M0.isNewNode = true; % flag, true if the current node is a new node
    M0.labelCount = 1; % label counter, assigned to the new node founded
end

% output message
if M0.isNewNode
    fprintf('New node %d\n', M0.stack(end));
else
    fprintf('Known node %d\n', M0.stack(end));
end

% log the path into the logbook
if ~isfield(M0,'logbook')
    c = mod(1, length(La) + 1); % for DFS, always choose the first rightmost branch
    M0.isNewNode = true;
    M0.labelCount = M0.labelCount + 1;
    M0.stack = [M0.stack M0.labelCount]; % push a new node 1 into the stack
    M0.logbook = [c length(La)+1];
else
    c = mod(1, length(La) + 1);
    M0.logbook = [M0.logbook ;[c length(La)+1] ];
    [~, path] = shortestPath(M0); % get the shortest path from start to current node. 
    % See the shortestPath.m for more information.
    
    %find out whether it's time to end the travelsal
    if size(path,1)>1
        if path(1,1)==0 && path(2,1)==0
            c = -1; % means that the simulation ends
            M1 = M0;
            return;
        end
    end
    
    if ~isempty(path)
        % find out whether it is backtracking on the tree
        if path(end,1) == 0 && size(path,1)>1
            M0.isNewNode = false;
            M0.stack(end) = [];
        else
            M0.isNewNode = true;
            M0.labelCount = M0.labelCount + 1;
            M0.stack = [M0.stack M0.labelCount];
        end
    end

end


M1 = M0;
