function Pn = get_0Pn(in1)
%GET_0PN
%    PN = GET_0PN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    09-Oct-2019 23:05:54

q1 = in1(1,:);
q2 = in1(2,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
Pn = reshape([0.0,0.0,0.0,t2,t4,0.0,t2+t2.*t3-t4.*t5,t4+t2.*t5+t3.*t4,0.0],[3,3]);
