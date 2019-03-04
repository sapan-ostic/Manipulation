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
run_symbolic;

%% Plot all interim steps of optimization
View_Interim_Steps = 1;  % Toggle 1 to view 

%% Forward Kinematics
Test_q = (pi/180)*[30,90]';

% Get joint positions
des_Pn = get_0Pn(Test_q);
T_all = get_0Tn(Test_q);
Tsd = T_all(:,:,end);

% Plotting 
figure(1);
plot(des_Pn(1,:),des_Pn(2,:),'-o','LineWidth',4);
axis equal;
xlabel('X'), ylabel('Y'); 

%% Inverse Kinematics using Newton-Raphson Method

% Initial guess for optimization
q_guess = (pi/180)*[0 0]';
th_ = q_guess;

eps_w = 0.001;
eps_v = 1e-4;
w_norm = 100*eps_w;
v_norm = 100*eps_v;

count = 1;

while (w_norm > eps_w) || (v_norm > eps_v)
   
    %% Transformation matrix of Body in Spatial frame 
    T_all = get_0Tn(th_);
    Tsb = T_all(:,:,end); 
    
    %% Finding Adjoint map space <== body
    Rsb = Tsb(1:3,1:3);
    Psb = Tsb(1:3,4);
    
    Psb = [  0   ,-Psb(3) , Psb(2) ; ...
           Psb(3),    0   ,-Psb(1) ; ...
          -Psb(2), Psb(1) ,   0    ; ];
    
    Ad_Tsb1 = [   Rsb   , zeros(3)];        
    Ad_Tsb2 = [ Psb*Rsb ,   Rsb   ];
    
    Ad_Tsb = [Ad_Tsb1; Ad_Tsb2]; 
    
    %% Body Twist
    Tbd = Tsb\Tsd;         % Tbd = Tbs*Tsd ; Tbs = inv(Tsb)
    Vb = logm(Tbd);
    
    wx = Vb(3,2);
    wy = Vb(1,3);
    wz = Vb(2,1);
    vx = Vb(1,4);
    vy = Vb(2,4);
    vz = Vb(3,4);
    
    w = [wx,wy,wz];
    v = [vx,vy,vz]; 
    
    w_norm = norm(w);
    v_norm = norm(v);
    
    Vb = [wx, wy, wz, vx, vy, vz]';
    
    %% Spatial Twist
    Vs = Ad_Tsb*Vb;
    
    %% Cordinate/Spatial Jacobian
    Jso  = get_Jo(th_);
    Jsp  = get_Jp(th_);
    Js = [Jso;Jsp];
    
    %% Body Jacobian
    Jb = Ad_Tsb\Js;      
    
    %% Newton Raphson Iteration in Body Frame
%     th_new = th_ + 0.3*pinv(Jb)*Vb;
    
    %% Newton Raphson Iteration in Spatial Frame
    th_new = th_ + 0.3*pinv(Js)*Vs;
     
    th_ = th_new;
    
    % View all Iterations
    if(View_Interim_Steps == 1)
        des_Pn = get_0Pn(th_);
        
        figure(1)
        hold on;
        plot(des_Pn(1,:),des_Pn(2,:),'--o','LineWidth',1);
    end
    count = count + 1;
end   

disp(['Optimization Converged! Total iterations = ', num2str(count)]);

%% View Solution 
figure(1)
hold on;
plot(des_Pn(1,:),des_Pn(2,:),'--o','LineWidth',4);
