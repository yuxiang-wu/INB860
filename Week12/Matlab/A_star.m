function cheapest_path = A_star(rows, cols, grey)
    % initialize the cost matrix
    cost = zeros(length(rows), length(cols));
    for i = 1 : length(rows)
        for j = 1 : length(cols)
            if(grey(i, j))
                cost(i, j) = rows(i) + cols(j);
            else
                cost(i, j) = max(rows(i), cols(j));
            end
        end
    end
    cost(length(rows), length(cols)) = 0;
    
    heur = heuristic_function(cost);

    cell = [];
    cell.row = 0;
    cell.col = 0;
    cell.weight = heur(0, 0);
    
    
end