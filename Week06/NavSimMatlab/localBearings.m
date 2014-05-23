function [La , Li] = localBearings(G, n0, n1)
% Compute La, the list of the relative angles of the neighbours of n1 coming from n0
%  Li is the list of id's (labels) of the neighbours in the same order as La
%  The branches are ordered from negative relative angle to positive relative angle
%  That is, anti-clockwise

% created by f.maire@qut.edu.au  on the 26th April 2012


% G.xy = [0 0 ; 1 0 ; 2 0; 2 1;1 1;0 1; 1 2; 2 2];
% G.Neighb{1} = [2 5 6];

    
u = G.xy(n1,:) -G.xy(n0,:);
u = u/norm(u);  % normalized
u90 = [-u(2) u(1)];  % rotate by 90 deg anticlockwise

Ln = setdiff(G.Neighb{n1},[n0]); % list of the neighbours of n1 apart n0

%Ln


if isempty(Ln)
   La = [];
   Li=[];
   return
end

V = G.xy(Ln,:) - repmat(G.xy(n1,:), length(Ln),1);

V_lx = V * u' ;  % relative x-coord of the neighbours in the frame u,u90
V_ly = V * u90' ;  % relative y-coord of the neighbours in the frame u,u90

[La , ii] = sort(atan2(V_ly, V_lx) );

Li = Ln(ii); 


