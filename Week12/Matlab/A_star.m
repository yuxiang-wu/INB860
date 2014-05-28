function cheapest_path = A_star(rows_in, cols_in, grey)
    global rows;
    rows = rows_in;
    global cols;
    cols = cols_in;
    
    global numOfRow;
    numOfRow = length(rows);
    global numOfCol;
    numOfCol = length(cols);

    % initialize the cost_mat matrix
    cost_mat = zeros(numOfRow, numOfCol);
    for i = 1 : length(rows)
        for j = 1 : length(cols)
            if(grey(i, j))
                cost_mat(i, j) = rows(i) + cols(j);
            else
                cost_mat(i, j) = max(rows(i), cols(j));
            end
        end
    end
    cost_mat(1, 1) = 0;
    
    % initialize explored
    explored = zeros(numOfRow, numOfCol);
    
    % initialize the heuristic function
    h_mat = zeros(numOfRow, numOfCol);
    [~, h_mat] = heuristic(numOfRow, numOfCol, cost_mat, h_mat);

    % start the A* algorithm
    frontier = [numOfRow numOfCol 0];
    frontier_handle = zeros(numOfRow, numOfCol);
    %frontier_handle(numOfRow, numOfCol) = 1;
    
    while(~isempty(frontier))
        % pop the minimum element out of frontier
        [p, I] = min(frontier(:, 3), [], 1);
        s = frontier(I,:);
        frontier(I,:) = [];
        %frontier_handle(s(1), s(2)) = 0;
        
        % if it is goal then break
        if(s(1) == 1 && s(2) == 1)
            break;
        end
        
        % add s to explored
        explored(s(1), s(2)) = 1;
        frontier_handle(s(1), s(2)) = 0;
        
        % actions
        act = [1 0; 0 1; -1 0; 0 -1];
        for i = 1 : length(act)
            successor = s(1:2) + act(i,:);
            
            if(successor(1) == 0 || successor(1) > numOfRow || successor(2) == 0 || successor(2) > numOfCol)
                continue;
            end
            
            if( explored(successor(1), successor(2)) )
                continue;
            end
            
            %succ_handle = frontier_handle(successor(1), successor(2));
            [~,succ_handle]=ismember(successor(1:2),frontier(:,1:2),'rows');
            if( succ_handle )
                succ_priority = frontier(succ_handle, 3);
                cur_priority = p + cost_mat(successor(1), successor(2)) + h_mat(successor(1), successor(2)) - h_mat(s(1), s(2));
                if(cur_priority < succ_priority)
                    frontier(succ_handle, 3) = cur_priority;
                end
            else
                cur_priority = p + cost_mat(successor(1), successor(2)) + h_mat(successor(1), successor(2)) - h_mat(s(1), s(2));
                frontier = [frontier; successor cur_priority];
                frontier_handle(successor(1), successor(2)) = 1;
            end
        end
    end
end

function [x, y] = plotAtCenter(r, c)
    x = sum(cols(1:c)) - cols(c) / 2;
    y = sum(rows(1:r)) - rows(r) / 2;
end