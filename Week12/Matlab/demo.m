function demo(row, col, grey)
    clf;
    hold on;
    
    row = fliplr(row);
    col = fliplr(col);
    plotGrid(row, col);
    
    if(nargin < 3)
        grey = rand(length(row), length(col)) > 0.7;
    end
    
    plotGreyPatch(row, col, grey);
    
    [cost_mat, cheapest_path] = A_star(row, col, grey);
    
    backtrack(row, col, cheapest_path, cost_mat);
    
    hold off
end