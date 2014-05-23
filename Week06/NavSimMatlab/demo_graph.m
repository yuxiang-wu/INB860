
%                      Graph navigation script

%  Script to demonstrate a topological or metric navigation strategies
%  Uncomment the relevant commands to run the different demos
% The script creates a tree 'T' and a graph 'G', then test different functions
% 

% Created by f.maire@qut.edu.au  on the 26th April 2012
% Revised on 8 March 2014


%---------------  T is structure representing a tree example  --------

%  T.xy is the matrix of 2D coordinates of the nodes of the graph
%  The ith row of T.xy is the x and y coordinates of the ith node
T.xy = [0 0 ; 1 0 ; 2 0; 2 1;1 1;0 1; 1 2; 2 2];

% T.Neighb{k} is the list of neighbours of the kth node
T.Neighb{1} = [2 5 6];
T.Neighb{2} = [1 3 4];
T.Neighb{3} = [2];
T.Neighb{4} = [2 7 8];
T.Neighb{5} = [1];
T.Neighb{6} = [1];
T.Neighb{7} = [4];
T.Neighb{8} = [4];

T.numVertices = length(T.Neighb);

% ------------------------------------------------------------------------




%--------  G is structure representing a planar graph  example -----------

%  G.xy is the matrix of 2D coordinates of the nodes of the graph
%  The ith row of G.xy is the x and y coordinates of the ith node
G.xy = [0 0 ; 1 0 ; 2 0; 2 1;1 1;0 1; 1 2; 2 2];

% G.Neighb{k} is the list of neighbours of the kth node
G.Neighb{1} = [2 5 6];
G.Neighb{2} = [1 3 ];
G.Neighb{3} = [2 4 5];
G.Neighb{4} = [3 5 8];
G.Neighb{5} = [1 3 4 6 7];
G.Neighb{6} = [1 5 7];
G.Neighb{7} = [5 6 8];
G.Neighb{8} = [4 7];

G.numVertices = length(G.Neighb);

% ------------------------------------------------------------------------


% driver  = @myBehaviorA3; 
% driver  = @myBehaviorA4; 
driver  = @myBehaviorA5; 
  
node0 = 2 ; node1 = 1; % initial position of the robot (on the edge node0-node1)

steps = 21 ; % number of iterations the simulation is run
 
% simGraphNavigation(G, driver, node0, node1, steps)

M = simGraphNavigation(T, driver, node0, node1, steps);
 
fprintf('Final map\n')
disp(M.logbook) % with TheseusCrawler
