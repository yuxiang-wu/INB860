function demo(row, col, grey)
    row = fliplr(row);
    col = fliplr(col);
    plotGrid(row, col);
    
    if(nargin < 3)
        grey = rand(length(row), length(col)) > 0.7;
    end
    
    result = A_star(row, col, grey);
end