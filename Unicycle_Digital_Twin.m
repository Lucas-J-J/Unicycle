clc; clear all; close all; fclose('all');
%Author: Carolyn Pethrick, December 2020-January 2021

%% User Input

%timing
%start time should usually be zero
t_start_global = 0;
%longest possible time. Program will abort if unicycle ends before this
t_end_global = 2;
%controller clock speed. Represents digital tick in controller.
t_interval_global = 0.01;
% change max step size in forward dynamics if changing this

%Initial torques
T_init = 0;
SF_init = 0.01;
RF_init = 0;
LF_init = 0;

%initial position
init_global = [0 0 pi/2 0 pi/2 0 0 1]';
% init format ["phi","phi'","theta","theta'", "alpha","alpha'","psi","psi'"]

% initial setpoint
init_setpoint_global = [0   pi/2    pi/2    0];

% geometry
%Dimensions from Hemanth 20.11.20
g.m = 2.948;
g.Ro = 0.1016;
g.Ip = 0.007849;
g.Iw = 0.01495;
g.mf = 2.326;
g.Ib = 0.007289;
g.If = 0.0445;
g.lf = 0.2523;
g.l = 0.5119;
g.W = g.m*9.81;
g.Wf = g.mf*9.81;
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

%header for history file
writematrix(['t', "phi","phi'","phi''", "theta","theta'", "theta''", "alpha","alpha'","alpha''", "psi","psi'","psi''", 'X', 'Y', 'Z'], hist_name)

%header for log file
fid = fopen(log_name, 'w');
fprintf(fid, "UNICYCLE DIGITAL TWIN\n");
fprintf(fid, "Run started at ");
fprintf(fid,  datestr(now, 'dd/mm/yy-HH:MM'));
fprintf(fid, "\n\nGeometry is:\n");
g.values = table2array(struct2table(g));
g.names = ["g.m ";"g.Ro";"g.Ip"; "g.Ip"; "g.mf"; "g.Ib"; "g.If"; "g.lf"; "g.l "; "g.W "; "g.Wf"; "g.Wr"];
for i=1:length(g.values)
    fprintf(fid, "\t%s\t=\t%.6f;\n", g.names(i), g.values(i));
end
fprintf(fid, "\nInitial torques are:\n");
fprintf(fid, "\tT\t=\t%.4f\n", T_init);
fprintf(fid, "\tSF\t=\t%.4f\n", SF_init);
fprintf(fid, "\tRF\t=\t%.4f\n", RF_init);
fprintf(fid, "\tLF\t=\t%.4f\n", LF_init);
fprintf(fid, "\nTiming settings are:\n");
fprintf(fid, "\tstart at %.4f\n", t_start_global);
fprintf(fid, "\tfinish at %.4f\n", t_end_global);
fprintf(fid, "\tsampling time is %.4f\n", t_interval_global);

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
T  = T_init;
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
%     T = T_init;
%     SF = SF_init;
%     RF = RF_init;
%     LF = LF_init;
     

    % PID controller constants. Format = [phi theta alpha psi]; 

    Kp = [2 20 20 20]; 
    Kd = [1 3.5 3.5 1]; 
    Ki = [0.5 0.5 0.5 0.5]; 
    
    % init format ["phi","theta","alpha","psi"] 
    % determining error in phi dirrection (setpoint - current angle) 
    phi_error =   [0 0 0];   % [direct error, derivative error, integral error] 
    theta_error = [0 0 0]; 
    alpha_error = [0 0 0]; 
    psi_error =   [0 0 0]; 
    %direct error 
    phi_error(1) = init_setpoint_global(1) - out(2);  
    theta_error(1) = init_setpoint_global(2) - out(5);   
    alpha_error(1) = init_setpoint_global(3) - out(8);  
    psi_error(1) = init_setpoint_global(4) - out(11);  
    %derivative error 
    if t_step ~= 2 
        phi_error(2) = (phi_error(1) - previous_phi_error) / t_interval_global; 
        theta_error(2) = (theta_error(1) - previous_theta_error) / t_interval_global; 
        alpha_error(2) = (alpha_error(1) - previous_alpha_error) / t_interval_global; 
        psi_error(2) = (psi_error(1) - previous_psi_error) / t_interval_global;
    end
    % ^^ doesn't compute derivative for first loop because previous_[angle]_error doesn't exist yet 
    %integral error 
    if t_step ~= 2 
        phi_error(3) = (phi_error(1) + previous_phi_error) / 2 * t_step + phi_error(3); 
        theta_error(3) = (theta_error(1) + previous_theta_error) / 2 * t_step + theta_error(3); 
        alpha_error(3) = (alpha_error(1) + previous_alpha_error) / 2 * t_step + alpha_error(3); 
        psi_error(3) = (psi_error(1) + previous_psi_error) / 2 * t_step + psi_error(3); 
    end
    % ^^ essentially midpoint numerical integration, then adding to previous integral error total 
    %setting previous error for next loop 
    previous_phi_error   = phi_error(1); 
    previous_theta_error = theta_error(1); 
    previous_alpha_error = alpha_error(1); 
    previous_psi_error   = psi_error(1); 
    % creating proportional torque response based on error 
    SF = (-phi_error(1) * Kp(1)) + (-phi_error(2) * Kd(1)) + (-phi_error(3) * Ki(1)); 
    RF = (theta_error(1) * Kp(2)) + (theta_error(2) * Kd(2)) + (theta_error(3) * Ki(2)); 
    LF = (-alpha_error(1) * Kp(3)) + (-alpha_error(2) * Kd(3)) + (-alpha_error(3) * Ki(3)); 
    T = (-psi_error(1) * Kp(4)) + (-psi_error(2) * Kd(4)) + (-psi_error(3) * Ki(4)); 
    
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
