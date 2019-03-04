%% UR5e Project
% Author: Sapan Agrawal
% Last Edited: 03 March, 2019 
% The code solves the inverse kinematics problem using: 
% Newton-Raphson Method

clear; close all; clc;

%% Select Manipulator
manipulator = 'UR5e';

%% Adding manipulator folder paths
path_to_rm = genpath('/home/sapanostic/Projects/Manipulators/Matlab/');
rmpath(path_to_rm);
path_to_add = genpath(['/home/sapanostic/Projects/Manipulators/Matlab/',manipulator]);
addpath(path_to_add);

% Loading manipulator constants
constants_script = [manipulator,'_constants'];
eval(constants_script);

% Generate functions
% run_symbolic;

%% Plot all interim steps of optimization
View_Interim_Steps = 0;  % Toggle 1 to view 

%% Forward Kinematics
Test_q = (pi/180)*[0,0,0,0,0,0]';

% Get joint positions
des_Pn = get_0Pn(Test_q);

% Plotting 
figure(1);
plot3(des_Pn(1,:),des_Pn(2,:),des_Pn(3,:),'-o','LineWidth',4);
axis equal;
xlabel('X'), ylabel('Y'), zlabel('Z'); 

%% Inverse Kinematics using Newton-Raphson Method

% Desired end-effector SE(3) position
xd = des_Pn(:,end);

% Initial guess for optimization
q_guess = (pi/180)*[0 0 10 0 0 0]';
th_ = q_guess;

eps = 0.001;
del = 100 * eps;
count = 1;

while del>eps
    Jp  = get_Jp(th_);
    
    des_Pn = get_0Pn(th_);
    f_th = des_Pn(:,end);
    
    th_new = th_ + pinv(Jp)*(xd - f_th);
    
    del = norm(xd-f_th);
    th_ = th_new;
    
    % View all Iterations
    if(View_Interim_Steps == 1)
        des_Pn = get_0Pn(th_);
        
        figure(1)
        hold on;
        plot3(des_Pn(1,:),des_Pn(2,:),des_Pn(3,:),'--o','LineWidth',1);
        axis equal
        xlabel('X'), ylabel('Y'),zlabel('Z')
    end
    count = count + 1;
end    

disp(['Optimization Converged! Total iterations = ', num2str(count)]);

%% View Solution 
figure(1)
hold on;
plot3(des_Pn(1,:),des_Pn(2,:),des_Pn(3,:),'--o','LineWidth',4);
axis equal
xlabel('X'), ylabel('Y')


