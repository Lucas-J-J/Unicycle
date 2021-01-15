clc; clear all; close all; fclose('all');
%Author: Carolyn Pethrick, December 2020

%% User Input

%timing
%start time should usually be zero
t_start_global = 0;
%longest possible time. Program will abort if unicycle ends before this
t_end_global = 2;
%controller clock speed. Represents digital tick in controller.
t_interval_global = 0.01; 

%%Initial torques
T_init = 0;
SF_init = 0.01;
RF_init = 0;
LF_init = 0;

%initial position
init_global = [0 0 pi/2 0 pi/2 0 0 2*pi]';
% init format ["phi","phi'","theta","theta'", "alpha","alpha'","psi","psi'"]

% geometry
% 28.12.20 NOT CURRENTLY IMPLEMENTED
%Dimensions from Hemanth 20.11.20
g.m = 2.948;
g.Ro = 0.1016;
g.W = g.m*9.81;
g.Ip = 0.007849;
g.Iw = 0.01495;
g.mf = 2.326;
g.Ib = 0.007289;
g.If = 0.0445;
g.Wf = g.mf*9.81;
g.lf = 0.2523;
g.l = 0.5119;
g.Wr =0*9.81; %no rider


%% Setup: Preprocessing data

t_arr = t_start_global:t_interval_global:t_end_global;

%% Setup: Initiating files

control_name = 'solution_files/control.txt';
out_name = 'solution_files/out.txt';
log_name = 'solution_files/log.txt';
hist_name = 'solution_files/history.txt';
geometry_name = 'solution_files/geometry.mat';

%clear solution_files folder to remove previous run
answer = input("Do you wish to overwrite ./solution_files/* ? [Y/N]  ", 's');
if ~strcmpi(answer, 'Y')
    error("Aborting program. Please move any files from ./solution_files/")
end
%removing using windows command line
!del /q solution_files\*

%writing geometry to .mat file
save('solution_files/geometry.mat', 'g')

%header for log file
fid = fopen(log_name, 'w');
fprintf(fid, "UNICYCLE DIGITAL TWIN\n");
fprintf(fid, "Run started at ");
fprintf(fid,  datestr(now, 'dd/mm/yy-HH:MM'));

%header for history file
writematrix(['t', "phi","phi'","phi''", "theta","theta'", "theta''", "alpha","alpha'","alpha''", "psi","psi'","psi''", 'X', 'Y', 'Z'], hist_name)

%ADD: full run settings: geometry, initial conditions, timing. 

fprintf(fid, "\nStarting run\n");
fprintf(fid, "*********************************************\n\n");
fclose(fid);

%% RUN

%First time step
t_step = 1;
t_start = t_arr(1);
t_interval = t_interval_global;
init = init_global;
% initial torques defined above
T = T_init;
SF = SF_init;
RF = RF_init;
LF = LF_init;
writematrix([t_step; t_start; t_interval; init; T; SF; RF; LF; 0;0;0], control_name)
%run unicycle dynamics file
run('internal/UnicycleDynamics.m')
%UnicycleDynamics.m will create out.txt and append to history.txt and
%log.txt

%Subsequent timesteps
for t_step = 2:length(t_arr)-1
    %timing
    t_start = t_arr(t_step);
    t_end = t_arr(t_step+1);
    t_interval = t_interval_global; %this assignment assumes constant interval
    
    %reading out.txt
    out = readmatrix(out_name);
    
    %breaking if unicycle is toppled
    if all(out==0)
        warning('Unicycle has toppled before specified end time.');
        fprintf('Stopping run.\n')
        break;
    end
    
    recorded_t_end = out(1);
    phi = out(2:4);
    theta = out(5:7);
    alpha = out(8:10);
    psi = out(11:13);
    pathXYZ = out(14:16);
    
    %%%%%%%%%%%%%%%%%%CONTROLLER PROGRAMMING HERE%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Here put any feedback control. As a placeholder, I will use the
    % initial torques, i.e. feed-forward control with constant values.
    
    %controller should calculate new torque signal (T, SF, RF, LF).
    T = T_init;
    SF = SF_init;
    RF = RF_init;
    LF = LF_init;
    %%%%%%%%%%%%%%%%%END CONTROLLER%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writing new control file
    init = [phi(1:2); theta(1:2); alpha(1:2); psi(1:2)];
    writematrix([t_step; t_start; t_interval; init; T; SF; RF; LF; pathXYZ], control_name);
    
    %run dynamics
    run('internal/UnicycleDynamics.m')
    %out.txt will be overwritten; history.txt and log.txt will be appended
    %to
end

%% Basic plotting

history = readmatrix(hist_name);

t = history(:, 1);
phi = history(:, 2:4);
theta = history(:, 5:7);
alpha = history(:, 8:10);
psi = history(:, 11:13);
pathXYZ = history(:, 14:16);

%plotting kinematics
figure()
subplot(2,1,1)
hold on
plot(t, psi(:,1))
plot(t, phi(:,1))
title("Unicycle Eulerian Angles")
ylabel("Angle [rad]")
legend(["Psi","Phi"])
subplot(2,1,2)
hold on;
plot(t, theta(:,1))
plot(t, alpha(:,1))
legend(["Theta",  "Alpha"])
ylabel("Angle [rad]")
xlabel("Time [s]")

%plotting path
figure()
plot(pathXYZ(:,1), pathXYZ(:,2))
hold on
plot(0,0, "r*")
legend(["Path of wheel center of mass", "Starting point"])
xlabel("x [m]")
ylabel("y [m]")

