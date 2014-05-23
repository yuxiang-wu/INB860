function fig_out = plot_graph(G, fig_in, titleMsg)
% Plot the graph G

% 
    if nargin<3
        titleMsg = 'Graph';
    end
        
    if nargin<2
        fig_out = figure;
    else
        fig_out = fig_in;
    end    
    figure(fig_out)
    
    

    if ~isfield(G,'nodeColor')
        for i = 1:G.numVertices
            G.nodeColor{i} = 'black';
        end
    end

        
    for i = 1:G.numVertices
        for j = G.Neighb{i}
            plot(G.xy([i j],1)' , G.xy([i j],2)' ,'-g','LineWidth',3)
            hold on
        end
    end

    for i = 1:G.numVertices
        text (G.xy(i,1), G.xy(i,2), int2str(i),'FontSize', 20,'color',G.nodeColor{i});
    end


    axis([-1+min(G.xy(:,1)) 1+max(G.xy(:,1))  -1+min(G.xy(:,2)) 1+max(G.xy(:,2))])
    axis off
    title(titleMsg)

    hold off

end

