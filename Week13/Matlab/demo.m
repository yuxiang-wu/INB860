function I = demo(p1, p2, p3, p4, t)
% p1, p2, p3, p4 are four training patterns, t is testing pattern
    % plot the inputs
    subplot(2,2,1);
    plotPattern(p1);
    subplot(2,2,2);
    plotPattern(p2);
    subplot(2,2,3);
    plotPattern(p3);
    subplot(2,2,4);
    plotPattern(p4);
    
    figure;
    plotPattern(t);
    
    % transform into 12 directions
    p1 = ceil(p1 / 30);
    p2 = ceil(p2 / 30);
    p3 = ceil(p3 / 30);
    p4 = ceil(p4 / 30);
    t = ceil(t / 30);
    
    % compute the edit distance between test and all other patterns
    dist = zeros(length(t)+1,length(p1)+1);
    d1 = compute_edit_dist(t, p1, dist);
    
    dist = zeros(length(t)+1,length(p2)+1);
    d2= compute_edit_dist(t, p2, dist);
    
    dist = zeros(length(t)+1,length(p3)+1);
    d3 = compute_edit_dist(t, p3, dist);
    
    dist = zeros(length(t)+1,length(p4)+1);
    d4 = compute_edit_dist(t, p4, dist);
   
    % find the minimum among them
    [d1 d2 d3 d4]
    [~, I] = min([d1 d2 d3 d4]);
end
