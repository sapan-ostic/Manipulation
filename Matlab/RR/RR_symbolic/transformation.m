function T = transformation(in1)
%TRANSFORMATION
%    T = TRANSFORMATION(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Mar-2019 00:35:31

a = in1(:,1);
alpha = in1(:,2);
d = in1(:,3);
th = in1(:,4);
t2 = sin(th);
t3 = cos(th);
t4 = cos(alpha);
t5 = sin(alpha);
T = reshape([t3,t2,0.0,0.0,-t2.*t4,t3.*t4,t5,0.0,t2.*t5,-t3.*t5,t4,0.0,a.*t3,a.*t2,d,1.0],[4,4]);
