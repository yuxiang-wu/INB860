function [ result, h_mat ] = heuristic( r, c, cost_mat, h_mat )

    if r < 1 || c < 1  % the heuristic of outbound cell is infinity
        result = Inf; % INFINITY
        return;
    elseif r == 1 && c == 1 % heuristic for goal is 0
        result = 0;
        return;
    else
        if h_mat(r, c) ~= 0 % use the cached value
            result = h_mat(r, c);
            return;
        else % if there is no cached value, compute recursively
            [upper, h_mat] = heuristic(r-1,c, cost_mat, h_mat);
            [left, h_mat] = heuristic(r,c-1, cost_mat, h_mat);
            result = cost_mat(r,c) + min(upper, left);
            h_mat(r, c) = result;
            return;
        end
    end

end

