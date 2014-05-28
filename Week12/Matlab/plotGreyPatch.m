function plotGreyPatch(rows, cols, grey)
    for i = 1 : length(rows)
        for j = 1 : length(cols)
            if(grey(i, j) && ~((i==1 && j==1) || (i==length(rows) && j==length(cols))))
                plotAtCenter(rows, cols, i, j);
            end
        end
    end
end

function [x, y] = plotAtCenter(rows, cols, r, c)
    x = sum(cols(1:c)) - cols(c) / 2;
    y = sum(rows(end:-1:r)) - rows(r) / 2;
    plot(x, y, 's', 'MarkerSize',30, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
end