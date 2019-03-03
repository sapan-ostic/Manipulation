%% Manipulator Symbolic 
% Author: Sapan Agrawal
% Last Edited: 03 March, 2019 
% The code all basic kinematics functions.

%%
syms alpha a d th
dh = [a alpha d th];

path = [manipulator,'/',manipulator,'_symbolic/'];
addpath(path);

%% Transformation 
% Oussamma Khatib
% T1 = [cos(th), -sin(th), 0 a];
% T2 = [sin(th)*cos(alpha), cos(th)*cos(alpha), -sin(alpha), -sin(alpha)*d];
% T3 = [sin(th)*sin(alpha), cos(th)*sin(alpha),  cos(alpha),  cos(alpha)*d];

T1 = [cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha) ,  a*cos(th)];
T2 = [sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha),  a*sin(th)];
T3 = [  0    ,     sin(alpha)     ,      cos(alpha)    ,      d    ];
T4 = [0, 0, 0, 1];
T = [T1;T2;T3;T4];

matlabFunction(T,'File',[path,'transformation'],'Vars',{dh});

%% Forward Kinematics 0Tn
q = sym('q', [n 1]);
Dh = sym(Dh) ;
Dh(:,4) = Dh(:,4) + q;

T_base(:,:,1) = sym(eye(4));

for i = 1:n
    T(:,:,i) = transformation(Dh(i,:));
    T_base(:,:,i+1) = T_base(:,:,i)*T(:,:,i); 
end
matlabFunction(T ,'File',[path,'get_Ti'],'Vars',{q});
matlabFunction(T_base,'File',[path,'get_0Tn'],'Vars',{q});

%% Joint Positions
O = sym([0 0 0 1])';

Pn(:,1) = sym([0, 0, 0 ,1])';

for i=1:n
    Pn(:,i+1) = T_base(:,:,i+1)*O;
end
Pn = Pn(1:3,:);
matlabFunction(Pn,'File',[path,'get_0Pn'],'Vars',{q});

%% Jacobian Jp

for i = 1:n
    Jp(:,:,i) = jacobian(Pn(1:3,i+1),q);
end

% Passing only the Jacobian corresponding to end effector
Jpn = Jp(:,:,n); 

matlabFunction(Jpn,'File',[path,'get_Jp'],'Vars',{q});

%% Jacobian Jo
Jo = sym(zeros(3,n,n));
Jo(:,1,1) = T_base(1:3,3,1);

for i = 2:n
    Jo(:,:,i) = Jo(:,:,i-1);
    Jo(:,i,i) = T_base(1:3,3,i) ;
end

% Passing only the Jacobian corresponding to end effector
Jon = Jo(:,:,n);  

matlabFunction(Jon,'File',[path,'get_Jo'],'Vars',{q});

%% Geometric Jacobian - Jn
Jn = [Jp(:,:,n); Jo(:,:,n)]; 

matlabFunction(Jn,'File',[path,'get_0Jn'],'Vars',{q});  

clear all