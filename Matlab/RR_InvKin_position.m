%% RR Manipulator Project
% Author: Sapan Agrawal
% Last Edited: 03 March, 2019 
% The code solves the inverse kinematics problem using: 
% Newton-Raphson Method

clear; close all; clc;

%% Select Manipulator
manipulator = 'RR';

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
Test_q = (pi/180)*[45,45]';

% Get joint positions
Pn = get_0Pn(Test_q);
xd = Pn(:,end);

% Plotting 
figure(1);
plot(Pn(1,:),Pn(2,:),'-o','LineWidth',4);
axis equal;
xlabel('X'), ylabel('Y'); 

%% Inverse Kinematics using Newton-Raphson Method
       
% Initial guess for optimization
q_guess = (pi/180)*[0 10]';
th_ = q_guess;

eps = 0.001;
del = 100 * eps;
count = 1;    

while del>eps
    
    % View all Iterations
    if(View_Interim_Steps == 1 || count==1)
        Pn = get_0Pn(th_);
        
        figure(1)
        hold on;
        plot(Pn(1,:),Pn(2,:),'--o','LineWidth',1);
    end
    
    Jp  = get_Jp(th_);
    
    Pn = get_0Pn(th_);
    f_th = Pn(:,end);
    
    th_new = th_ + pinv(Jp)*(xd - f_th);
    
    del = norm(xd-f_th);
    th_ = th_new;
  
    count = count + 1;
end 

disp(['Optimization Converged! Total iterations = ', num2str(count)]);

%% View Solution 
figure(1)
hold on;
plot(Pn(1,:),Pn(2,:),'--o','LineWidth',4);