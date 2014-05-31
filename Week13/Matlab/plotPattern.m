function plotPattern(ang)
    ang = ang * pi / 180;
    A = sin(ang);
    B = cos(ang);
    len = 5;
    x1 = 0; y1 = 0;
    
    hold on;
    
    for i = 1 : length(ang)
        x2 = x1 + len * B(i);
        y2 = y1 + len * A(i);
        plot([x1 x2], [y1 y2]);
        axis equal tight;
        x1 = x2;
        y1 = y2;
    end
    
    hold off;
end