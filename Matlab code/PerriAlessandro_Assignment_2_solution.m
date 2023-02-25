%% ROBOT DYNAMICS AND CONTROL ASSIGNMENT 2 %%%%
%% Perri Alessandro - s4476726
% Main code with values of actuation torques for each exercise and snapshot

clc
clear

n_exercises=3; %number of exercises
n_snapshot=2; %number of motion snapshot for each exercise
%inizialization of actuation torques vector
tau=cell(n_exercises,n_snapshot,2); %third index is "1" in absence of gravity and "2" otherwise
%%Exercise 2.1
run("es2_1.m")
tau{1,1,1}=invDyn(robot,F_ext,M_ext,zeros(3,1));
tau{1,1,2}=invDyn(robot,F_ext,M_ext,g);
%%Exercise 2.2
run("es2_2.m")
tau{1,2,1}=invDyn(robot,F_ext,M_ext,zeros(3,1));
tau{1,2,2}=invDyn(robot,F_ext,M_ext,g);
%%Exercise 3.1
run("es3_1.m")
tau{2,1,1}=invDyn(robot,F_ext,M_ext,zeros(3,1));
tau{2,1,2}=invDyn(robot,F_ext,M_ext,g);
%%Exercise 3.2
run("es3_2.m")
tau{2,2,1}=invDyn(robot,F_ext,M_ext,zeros(3,1));
tau{2,2,2}=invDyn(robot,F_ext,M_ext,g);
%%Exercise 4.1
run("es4_1.m")
tau{3,1,1}=invDyn(robot,F_ext,M_ext,zeros(3,1));
tau{3,1,2}=invDyn(robot,F_ext,M_ext,g);

clear F_ext M_ext g robot