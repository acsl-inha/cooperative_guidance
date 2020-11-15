clearvars;
close all;

%% Common Constants
Sim_D2R = pi/180;
Sim_R2D = 1/Sim_D2R;
Sim_G = 9.80665;

%% Simulation Parameters
Sim_Tf = 30;                                                               % [sec]
Sim_dt = 1e-3;                                                             % [sec]
Sim_time = 0 : Sim_dt : Sim_Tf;

%% Target Initial Parameters
TGT_pos_NED0 = [500 500 500];                                                 % [m]
TGT_HED0 = 90;                                                            % [deg]
TGT_FPA0 = 0;                                                             % [deg]
TGT_vel0 = 200;                                                            % [m/s]
TGT_TurnTime = [1 1.5 3 5];                                                % [sec]
TGT_HEDRate = [30 -30 50 90];                                            % [deg/s]
TGT_FPARate = [0 90 -40 20];                                            % [deg/s]
TGT_BW = 1;                                                               % [rad/s]

%% Missile Initial Parameters
MSL_pos_NED0 = [0 0 0];                                                    % [m]
MSL_vel0 = 300;                                                            % [m/s]
MSL_HED0 = 90;                                                            % [deg]
MSL_FPA0 = 0;                                                             % [deg]
MSL_Mass = 10;                                                             % [kg]
MSL_nav_gain = 4;                                                          % [-]
MSL_Moment_Jxyz = [1 10 10];                                            % [kg*m^2]
MSL_Euler_kp = 100*MSL_Moment_Jxyz;                                          % [-] BW = 10
MSL_Euler_kv = 1*sqrt(MSL_Euler_kp.*MSL_Moment_Jxyz);                    % [-] damp = 0.5
MSL_FOV = 3;                                                            % [deg]

Disp_Body_axlength = 30;
Disp_Body_Size = [30 9 9];
Disp_Thrust_DCS_Scale = 0.02;
Disp_Thrust_ACS_Scale = 5;

%% Target and Missile Trajectory
submodule_missile;

%% Plot Results
submodule_plot;