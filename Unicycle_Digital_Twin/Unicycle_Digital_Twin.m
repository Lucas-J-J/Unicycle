clc; clear all; close all; fclose('all');
%Author: Carolyn Pethrick, December 2020-January 2021

%% User Input

%timing
%start time should usually be zero
t_start_global = 0;
%longest possible time. Program will abort if unicycle ends before this
t_end_global = 3;
%controller clock speed. Represents digital tick in controller.
t_interval_global = 0.01;           % <change max step size in forward dynamics if changing this
%control system delay time (seconds)
delay_time = 0;

%Initial torques
T_init  = 0;
SF_init = 0;
RF_init = 70;
LF_init = 0;

%initial position
init_global = [0 0 pi/2 0 pi/2 0 0 0]';
% init format ["phi (yaw)","phi'","theta (roll)","theta'", "alpha (pitch)","alpha'","psi (wheel)","psi'"]

%initial setpoint
init_setpoint_global = [0 pi/2 pi/2 0];


% geometry
%Dimensions from Hemanth 20.11.20
g.m = 2;          %wheel mass (kg)        
g.Ro = 0.1016;    %radius of the wheel (m)
g.Ip = 0.00569;   %Moment of inertia of the wheel                     (yaw perspective)
g.Iw = 0.0101;    %wheel inertia about axle  0.0101;
g.mf = 20.735;    %mass of the whole thing minus the wheel
g.Ib = 0.4158;    %moment of the entire thing in yaw                  (minus wheel)
g.If = 0.3651;    %moment of inertia in pitch direction               (about centroid)               *pitch vs roll is way different!
g.lf = 0.3569;    %length from axle to fork                           center of mass (m)
g.l = 0.5334;     %length of fork (m)                                       *why?

g.W = g.m*9.81;   %internal calculation
g.Wf = g.mf*9.81; %internal calculation
g.Wr =0*9.81;     %no rider

I_gyro = 1;     %polar inertia of the gyroscope (kg m^2) 
w_gyro = 56.54;     %rotation speed of gyro (rad/s)


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

    %%%%%%%%%%%%%%%INITIAL CONTROLLER INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%
    % PID controller constants. Format = [phi theta alpha psi];
    Kp = [1 18.5904 160 130];
    Kd = [2.5 0 5 4];
    Ki = [0.3 0.002526 1 1.5];
    
    %converting controller delay time to equivalent step sizes
    delay_steps = round(delay_time/t_interval_global);

%     % initializing error vectors [direct error, derivative error, integral error]
%     phi_error =   [0 0 0];   
%     theta_error = [0 0 0];
%     alpha_error = [0 0 0];
%     psi_error =   [0 0 0];
    
    % initializing torque response "delay" vectors
    SF_delay = zeros(1,length(t_arr));
    RF_delay = zeros(1,length(t_arr));
    LF_delay = zeros(1,length(t_arr));
    T_delay  = zeros(1,length(t_arr));
    
    % initializing identification data
    SF_identification = zeros(1,length(t_arr)-1);       %(the 6 is to later cut off the first bit for better identification)
    RF_identification = zeros(1,length(t_arr)-1);
    LF_identification = zeros(1,length(t_arr)-1);
    T_identification = zeros(1,length(t_arr)-1);
    phi_yaw_identification = zeros(1,length(t_arr)-1); 
    theta_roll_identification = zeros(1,length(t_arr)-1);  
    alpha_pitch_identification = zeros(1,length(t_arr)-1); 
    psi_wheel_identification = zeros(1,length(t_arr)-1); 
    SF_identification(1) = SF_init;         %accounting for any initial "shoves" during the first timestep
    RF_identification(1) = RF_init;
    LF_identification(1) = LF_init;
    T_identification(1) = T_init;
    phi_yaw_identification(1) = init_global(1);     %setting first loop positions to their setpoints. 
    theta_roll_identification(1) = init_global(3);  
    alpha_pitch_identification(1) = init_global(5); 
    psi_wheel_identification(1) = init_global(7); 
    
    
    %%%%%%%%%%%%%%%END CONTROLLER INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    %%%%%%%%%%%%%%%%%%CONTROLLER LOOP PROGRAMMING HERE%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Here put any feedback control. As a placeholder, I will use the
    % initial torques, i.e. feed-forward control with constant values.
    
    %controller should calculate new torque signal (T, SF, RF, LF).
    
    %updating setpoints    [phi theta alpha psi]
    init_setpoint_global = [0 pi/2 pi/2 (out(8)-pi()/2) ];
    
        % initializing error vectors [direct error, derivative error, integral error]
    phi_error =   [0 0 0];   
    theta_error = [0 0 0];   
    alpha_error = [0 0 0];
    psi_error =   [0 0 0];
    
    
    %direct error (setpoint - current angle)
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
    SF_delay(t_step) = (-phi_error(1) * Kp(1)) + (-phi_error(2) * Kd(1)) + (-phi_error(3) * Ki(1));
    RF_delay(t_step) = (theta_error(1) * Kp(2)) + (theta_error(2) * Kd(2)) + (theta_error(3) * Ki(2));
    LF_delay(t_step) = (-alpha_error(1) * Kp(3)) + (-alpha_error(2) * Kd(3)) + (-alpha_error(3) * Ki(3));
    T_delay(t_step) = (-psi_error(1) * Kp(4)) + (-psi_error(2) * Kd(4)) + (-psi_error(3) * Ki(4));
    
    % implementing actual torque response (based on delay time)
     if (t_step - delay_steps) > 2
    SF = SF_delay(t_step - delay_steps);
    RF = RF_delay(t_step - delay_steps);
    LF = LF_delay(t_step - delay_steps);
    T  = T_delay(t_step - delay_steps);
    else
    SF = 0;
    RF = 0;
    LF = 0;
    T = 0;
     end
     
   % for system identification
    SF_identification(t_step) = SF;
    RF_identification(t_step) = RF;
    LF_identification(t_step) = LF;
    T_identification(t_step) = T;
    phi_yaw_identification(t_step) = out(2); 
    theta_roll_identification(t_step) = out(5);  
    alpha_pitch_identification(t_step) = out(8); 
    psi_wheel_identification(t_step) = out(11);  
  
    
    %%%%%%%%GYROSCOPE EFFECTS%%%%%%%%%  
    %T_gyro = I_gyro * w_gyro * angle_dot
    %theta(roll,RF) -> alpha(pitch,LF)
    %alpha(pitch,LF) -> theta(roll,RF)
    
    T_gyro_LF = I_gyro * w_gyro * out(6);
    T_gyro_RF = I_gyro * w_gyro * out(9);
    
%     LF = LF + T_gyro_LF;
%     RF = RF + T_gyro_RF;

    
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
legend(["Psi (wheel)","Phi (yaw)"])
subplot(2,1,2)
hold on;
plot(t, theta(:,1))
plot(t, alpha(:,1))
legend(["Theta (roll)",  "Alpha (pitch)"])
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

figure()
plot(t_arr,LF_delay)
hold on
plot(t_arr,SF_delay)
plot(t_arr,RF_delay)
plot(t_arr,T_delay)
legend('LF - Pitch Torque','SF - Yaw Torque','RF - Roll torque','T - Wheel Torque')
title('Actuation Torque Response');
xlabel('Time [s]')
ylabel('Torque [nm]')

%%%%%%%%%3D plotting%%%%%%%%%%
% figure()
% plot3(RF_delay,SF_delay,T_delay,'o')
