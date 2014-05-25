function cheapest_path = A_star(rows, cols, grey)
    numOfRow = length(rows);
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
    
    % initialize the heuristic function
    h_mat = zeros(numOfRow, numOfCol);
    [~, h_mat] = heuristic(numOfRow, numOfCol, cost_mat, h_mat);

    % start the A* algorithm
    frontier = [numOfRow numOfCol 0];
    while(~isempty(frontier))
        
    end
end