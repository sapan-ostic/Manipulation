%% UR5_constants
% addpath('/home/sapanostic/Projects/UR5e/Matlab/UR5e/UR5e_symbolic')

home_position = (pi/180)*[0, 0]';

%         [a      , alpha ,    d   , theta] 
          
Dh =      [   1   ,   0   ,    0   , 0 ;...
              1   ,   0   ,    0   , 0];          

Dh(:,4)  = Dh(:,4) + home_position;

n = length(Dh(:,1));