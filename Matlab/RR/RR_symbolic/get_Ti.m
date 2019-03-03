function T = get_Ti(in1)
%GET_TI
%    T = GET_TI(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Mar-2019 00:35:31

q1 = in1(1,:);
q2 = in1(2,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
T = reshape([t2,t4,0.0,0.0,-t4,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2,t4,0.0,1.0,t3,t5,0.0,0.0,-t5,t3,0.0,0.0,0.0,0.0,1.0,0.0,t3,t5,0.0,1.0],[4,4,2]);