function M = simGraphNavigation(G, driver, node0, node1, steps)
%  Simulate the navigation in the graph G with the robot controlled by the
%  function 'driver'
%  The function 'drunkGraphCrawler()' is an example of driver.
%
%  The driver function should implement the following interface
%        [c , M1] = driver(La, d , M0 )
%        where
%           inputs:
%              La :  list of the apparent angles in degrees anticlockwise of 
%                    the incident branches with respect to the branch the robot is on
%              d  :  noisy distance travelled on the last segment
%              M0 :  the current map of the robot
% 
%           outputs:
%               c :  branch choice or negative value to stop
%                    1 corresponds to the rightmost branch
%                    2 corresponds to the 2nd rightmost branch
%                    0 corresponds to a U turn
%              M1 : updated map
%      
%
%  The simulation starts with the robot on the edge node0->node1 with the robot's
%  arriving at node1.  The simulation is run for 'steps' iterations.
%  The simulation displays 'La' the list of sorted relative angles of the branches at the current node and 
%  'Li'  the list of the corresponding neighbours labels 


% created by f.maire@qut.edu.au  on the 26th April 2012
% Revised on 8 March 2014

% n0: previous node
% n1: current node
n0 = node0; n1 = node1;

M = []; % initial empty map

fprintf('\n\n    *** Starting the simulator ***\n\n')

figG = plot_graph(G);

for step = 1:steps

   fprintf('SimGraphNavigation:: Coming from node %d arriving at node  %d \n', n0, n1)
   
   Xrobot = 0.8*G.xy(n1,:)+0.2*G.xy(n0,:);   
   figG = plot_graph(G,figG, ['from ' num2str(n0) ' to ' num2str(n1)]);   
   hold on
   plot(Xrobot(1),Xrobot(2),'*r')
%    title(['from ' num2str(n0) ' to ' num2str(n1)])
   hold off
   
   [La , Li] = localBearings(G, n0, n1);
   
   fprintf('SimGraphNavigation:: Angles  [ ')
   fprintf('%2.2f ',La)
   fprintf(']\n')
   
   fprintf('SimGraphNavigation:: Neighbours  [ ')
   fprintf('%2d ',Li)
   fprintf(']\n')
   
   [c, M] = driver(La, norm(G.xy(n1,:)-G.xy(n0,:)) , M);

   fprintf('SimGraphNavigation:: Driver chooses the %d th branch\n\n', c);
%    if ~isempty(M)
%        fprintf('Current map\n')
%        disp(M.logbook)
%    end
   
   if 0<c
       n0 = n1;
       n1 = Li(c);
   elseif c==0
       % U turn
       old_n0 = n0;
       n0 = n1;
       n1 = old_n0;
   else
      % c < 0
	  break
   end

   pause;
end

% fprintf('Final map\n')
% disp(M)





