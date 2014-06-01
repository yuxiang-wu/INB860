function plotGrid(Rows, Cols)
    
    height = sum(Rows);
    width = sum(Cols);
    
    % plot horizontal lines
    sumRow = 0;
    for i = length(Rows) : -1 : 1
        plot([0 width], [sumRow sumRow],'LineWidth',3, 'MarkerEdgeColor','k');
        sumRow = sumRow + Rows(i);
    end
    
    % plot vertical lines
    sumCol = width;
    for i = length(Cols) : -1 : 1
        plot([sumCol sumCol], [0 height],'LineWidth',3, 'MarkerEdgeColor','k');
        sumCol = sumCol - Cols(i);
    end
    
    plot([0 0],[0 height - Rows(1)],'LineWidth',3, 'MarkerEdgeColor','k');
    plot([Cols(1) width],[height height],'LineWidth',3, 'MarkerEdgeColor','k');

    axis equal; % x,y axis will be at same scale
end
