function [result, result_dist] = compute_edit_dist(test, train, dist)
    % cost for delete and insert
    delete = 3;
    insert = 3;

    test_len = length(test);
    train_len = length(train);
    
    % use the cached value
    if(dist(test_len + 1, train_len + 1) ~= 0)
        result = dist(test_len + 1, train_len + 1);
        result_dist = dist;
        return;
    end
    
    % if no cached value found, compute edit distance recursively
    if(test_len == 0 && train_len == 0)
        result = 0;
    elseif(test_len > 0 && train_len == 0)
        [a, dist] = compute_edit_dist(test(1:end-1), train, dist);
        result = a + delete;
    elseif(test_len == 0 && train_len > 0)
        [b, dist] = compute_edit_dist(test, train(1:end-1), dist);
        result = b + insert;
    else
        [a, dist] = compute_edit_dist(test(1:end-1), train, dist);
        [b, dist] = compute_edit_dist(test, train(1:end-1), dist);
        [c, dist] = compute_edit_dist(test(1:end-1), train(1:end-1), dist);
        result = min([a + delete b + insert c + substitute(test(end), train(end))]);
    end
    
    % update the dist matrix
    dist(test_len + 1, train_len + 1) = result;
    result_dist = dist;
end

% cost for substitute
function result = substitute(c, d)
    result = min([abs(c-d) 12-abs(c-d)]);
end
