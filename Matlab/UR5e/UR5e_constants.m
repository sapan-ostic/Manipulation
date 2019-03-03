%% UR5_constants
% addpath('/home/sapanostic/Projects/UR5e/Matlab/UR5e/UR5e_symbolic')

home_position = 0*(pi/180)*[0, 135, 180, 0, 0 ,0]';

%         [a      ,alpha , d      , theta] 

Dh =      [0      , pi/2 ,0.1625  , 0;...
          -0.425  , 0    , 0      , 0;...
          -0.39225, 0    , 0      , 0;...
          0       , pi/2 , 0.1333 , 0;...
          0       ,-pi/2 , 0.0997 , 0;...
          0       , 0    , 0.0996 , 0];

Dh(:,4)  = Dh(:,4) + home_position;

n = length(Dh(:,1));