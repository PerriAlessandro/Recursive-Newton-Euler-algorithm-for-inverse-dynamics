% clc
% clear

%%Exercise 4.1

%============= Data =============%

%LINK 1
robot.Data(1).mass = 20.0; %mass [Kg]
robot.Data(1).poswrtparent = [0.0 0.0 0.0]'; %position wrt the previous frame [m]
robot.Data(1).CoM = [0.5 0.0 0.0]; %Center of Mass [m]
robot.Data(1).I = diag([0.2 0.2 0.8]); %MoI tensor [Kg*m^2]
robot.Data(1).axangrot = axang2rotm([0 0 1 0]); %fixed rotation wrt the previous frame
robot.Data(1).type = "revolute"; %type of joint

%LINK 2
robot.Data(2).mass = 20.0;  %mass [Kg]
robot.Data(2).poswrtparent = [1.0 0.0 0.0]'; %position wrt the previous frame [m]
robot.Data(2).CoM = [0.4 0.0 0.0];  %Center of Mass [m]
robot.Data(2).I = diag([0.2 0.2 0.8]); %MoI tensor [Kg*m^2]
robot.Data(2).axangrot = axang2rotm([1 0 0 pi/2]); %fixed rotation wrt the previous frame
robot.Data(2).type = "revolute"; %type of joint

%LINK 3
robot.Data(3).mass = 6.0;  %mass [Kg]
robot.Data(3).poswrtparent = [0.8 0.0 0.0]'; %position wrt the previous frame [m]
robot.Data(3).CoM = [0.35/2 0.0 0.0];  %Center of Mass [m]
robot.Data(3).I = diag([0.08 0.08 0.1]); %MoI tensor [Kg*m^2]
robot.Data(3).axangrot = axang2rotm([0 0 1 0]); %fixed rotation wrt the previous frame
robot.Data(3).type = "revolute"; %type of joint

%============= Config =============%
% robot.Config(i).q--> i-th Joint Position
% robot.Config(i).qd--> i-th Joint Velocity
% robot.Config(i).qdd--> i-th Joint Acceleration
%LINK 1
robot.Config(1).q = pi/9;
robot.Config(1).qd = 0.2;
robot.Config(1).qdd = 0.1;

%LINK 2
robot.Config(2).q = 2*pi/9;
robot.Config(2).qd = 0.15;
robot.Config(2).qdd = 0.085;

%LINK 3
robot.Config(3).q = pi/18;
robot.Config(3).qd = -0.2;
robot.Config(3).qdd = 0.0;



%External forces and moments
F_ext=zeros(3,3);
M_ext=zeros(3,3);
%gravity acceleration vector [m/s^2]
g=[0.0;0.0;-9.81];


