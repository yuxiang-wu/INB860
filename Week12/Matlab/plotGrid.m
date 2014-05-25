function plotGrid(Rows, Cols)
    hold on;
    
    height = sum(Rows);
    width = sum(Cols);
    
    sumRow = 0;
    for i = length(Rows) : -1 : 1
        plot([0 width], [sumRow sumRow],'LineWidth',3);
        sumRow = sumRow + Rows(i);
    end
    
    sumCol = width;
    for i = length(Cols) : -1 : 1
        plot([sumCol sumCol], [0 height],'LineWidth',3);
        sumCol = sumCol - Cols(i);
    end
    
    plot([0 0],[0 height - Rows(1)],'LineWidth',3);
    plot([Cols(1) width],[height height],'LineWidth',3);
    
    hold off;
end