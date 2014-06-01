function backtrack( rows, cols, path, cost_mat)
    r = 1;
    c = 1;
    act = [-1 0; 0 -1; 1 0; 0 1]; % four posible actions, reversed version of that in A_star to achieve backtrack
    sum = 0;
    
    while(r <= length(rows) && c <= length(cols))
        plotAtCenter(rows, cols, r, c);
        sum = sum + cost_mat(r, c);
        
        fprintf('Current position is row %d, column %d.\n', r, c);
        fprintf('Current sum of cost is %d.\n', sum);
        fprintf('Press enter to continue...\n\n');
        
        % make a move
        tmp = path(r, c);
        if(tmp)
            r = r + act(tmp, 1);
            c = c + act(tmp, 2);
        else
            break;
        end
        
        pause;
    end
end

% plot at center of row r, column c
function [x, y] = plotAtCenter(rows, cols, r, c)
    x = sum(cols(1:c)) - cols(c) / 2;
    y = sum(rows(end:-1:r)) - rows(r) / 2;
    plot(x, y, 'o', 'MarkerSize', 20, 'MarkerEdgeColor','r', 'MarkerFaceColor', 'r');
end
