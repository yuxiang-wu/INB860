function plotPattern(ang)
    ang = ang * pi / 180;
    A = sin(ang);
    B = cos(ang);
    len = 5; % assume that speed of robot is constant, each move is of constant length
    x1 = 0; y1 = 0;
    
    hold on;
    
    % plot the path
    for i = 1 : length(ang)
        % current position
        x2 = x1 + len * B(i);
        y2 = y1 + len * A(i);

        plot([x1 x2], [y1 y2]);
        axis equal tight;
        
        x1 = x2;
        y1 = y2;
    end
    
    hold off;
end
