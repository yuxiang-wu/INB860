function plotGrid(Rows, Cols)
    hold on;
    
    sumRow = 0;
    for i = 1 : length(Rows)
        sumCol = sum(Cols);
        for j = 1 : length(Cols)
            nextCol = sumCol - Cols(j);
            plot([sumCol nextCol], [sumRow sumRow],'LineWidth',3);
            sumCol = nextCol;
        end
        sumRow = sumRow + Rows(i);
    end
    
    sumCol = sum(Cols);
    for i = 1 : length(Cols)
        sumRow = 0;
        for j = 1 : length(Rows)
            nextRow = sumRow + Rows(j);
            plot([sumCol sumCol], [sumRow nextRow],'LineWidth',3);
            sumRow = nextRow;
        end
        sumCol = sumCol - Cols(i);
    end
    
    sumRow = sum(Rows);
    plot([0 0],[0 sumRow - Rows(end)],'LineWidth',3);
    plot([Cols(end) sum(Cols)],[sumRow sumRow],'LineWidth',3);
    
    hold off;
end