function Jon = get_Jo(in1)
%GET_JO
%    JON = GET_JO(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Mar-2019 00:00:47

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = sin(q1);
t3 = cos(q1);
t4 = cos(q2);
t5 = cos(q3);
t6 = sin(q2);
t7 = sin(q3);
t8 = cos(q4);
t9 = t3.*t4.*t5;
t10 = t9-t3.*t6.*t7;
t11 = sin(q4);
t12 = t3.*t4.*t7;
t13 = t3.*t5.*t6;
t14 = t12+t13;
t15 = cos(q5);
t16 = sin(q5);
t17 = t2.*t4.*t5;
t18 = t17-t2.*t6.*t7;
t19 = t2.*t4.*t7;
t20 = t2.*t5.*t6;
t21 = t19+t20;
t22 = t4.*t7;
t23 = t5.*t6;
t24 = t22+t23;
t25 = t4.*t5;
t26 = t25-t6.*t7;
Jon = reshape([0.0,0.0,1.0,t2,-t3,0.0,t2,-t3,0.0,t2,-t3,0.0,t10.*t11+t8.*t14,t8.*t21+t11.*t18,-t8.*t26+t11.*t24,t2.*t15-t16.*(t8.*t10-t11.*t14),-t3.*t15-t16.*(t8.*t18-t11.*t21),-t16.*(t8.*t24+t11.*t26)],[3,6]);