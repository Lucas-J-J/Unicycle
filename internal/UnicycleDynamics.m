function [] = UnicycleDynamics()
% clc; close all; clearvars;
% Author: Carolyn Pethrick, May 2020-January 2021

%% Solution Method Flags

soln_method = "forward";

simplify = "none";
    %Valid options: "upright" or "small angle" or "none"
    %"upright": assume theta=pi/2 and alpha=pi/2
    %"small angle": assume first term of Taylor expansion about pi/2
    %"none" (or any other value): use full, non-simplified expressions

write_csv = true;
controlname = '../solution_files/control.txt';
outname = '../solution_files/out.txt';
logname = '../solution_files/log.txt';
histname = '../solution_files/history.txt';


%% Motion inputs
% read motion from a file, which has been printed according to
% writematrix([t_step; t_start; t_interval; init; T; SF; RF; LF], controlname)

control_input = readmatrix(controlname);

t_step = control_input(1);
t_start = control_input(2);
t_interval = control_input(3);
init = control_input(4:11);
T = control_input(12);
SF = control_input(13);
RF = control_input(14);
LF = control_input(15);
position = control_input(16:18);

%% Geometry

%read from .mat file (directly load variable g)
load('../solution_files/geometry.mat', 'g')


%% Solution
%angles are stores as a matrices, consistent with t
%format of matrices: columns are timesteps, consistent with t
%rows are derivatives: ind. 1 is zeroth; ind. 2 is 1st (speed); ind. 3 is
%2nd (acceleration)

t_end = t_start + t_interval;


if strcmpi(soln_method, "forward")
    [SOLN,SOLN_key,t, toppled] = ForwardDynamics(T, SF, RF, LF, t_start, t_end, init, g, simplify);
    
    %processing for storing
    phi = SOLN(1:3,:);
    theta = SOLN(4:6,:);
    alpha = SOLN(7:9, :);
    psi = SOLN(10:12, :);
end

if toppled
    t_end = t(end);
end
%% Calculating path

[vXYZ, pathXYZ] = CalculatePath(t, phi, theta, alpha, psi, g, position);

%% Storing result in csv

if write_csv
    
    matrix_write = [t SOLN' pathXYZ'];
    % Leave this line commented if you are using R2019
    %writematrix(matrix_write(end, :)', outname, 'WriteMode', 'overwrite')
    
    % Leave this line commented if you are using R2020
    writematrix(matrix_write(end, :)', outname)


    if toppled
        %row of zeros indicates toppled unicycle
        % Leave this line commented if you are using R2019
        %writematrix(zeros(1,16)', outname, 'WriteMode', 'overwrite')
        
        % Leave this line commented if you are using R2020
        writematrix(zeros(1,16)', outname)
    end
    
    %writing history file
    % Leave this line commented if you are using R2019
    %writematrix(matrix_write, histname, 'WriteMode', 'append')
    
    % Leave this line commented if you are using R2020
    [r, c] = size(matrix_write);
    fid = fopen(histname, "a");
    for i=1: r
        for j=1: c
            fprintf(fid, "%0.15f", matrix_write(i, j));
            if j == c
                fprintf(fid, "\n");
            else
                fprintf(fid, ",");
            end
        end
    end
    fclose(fid);
    %writematrix(matrix_write, histname)
    
    %writing log file 
    fid = fopen(logname, "a");    
    fprintf(fid, "\t\t\tTimestep %d\n\n", t_step);
    fprintf(fid, "Start time = %.4f\tEnd time = %.4f\n", t_start, t_end);
    % SOMETHING ABOUT INPUT ONCE I ADD THAT
    fprintf(fid, "\nTorque input is:\n");
    fprintf(fid, "\tT\t=\t%.4f\n", T);
    fprintf(fid, "\tSF\t=\t%.4f\n", SF);
    fprintf(fid, "\tRF\t=\t%.4f\n", RF);
    fprintf(fid, "\tLF\t=\t%.4f\n", LF);
    fprintf(fid, "\nAt the last timestep, the solution is \n");
    fprintf(fid,"\tt\t=\t%.4f\n", t(end));  
    for i = 1:length(SOLN_key)
        fprintf(fid,"\t%s\t=\t%.4f\n", SOLN_key(i), SOLN(i, length(t)));
    end
    if toppled
        fprintf(fid, "Unicycle has toppled at %.4f s. Ending run.", t_end);
    end
    fprintf(fid, "\n\n\n");
    fclose(fid);
end
fprintf('Finished t_step %.0f\n', t_step)
end