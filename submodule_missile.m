%% Declare Variables
Sim_length                              = length(Sim_time);

TGT_TurnCounter                         = zeros(1, 1);
TGT_HED                                 = zeros(Sim_length, 1);
TGT_FPA                                 = zeros(Sim_length, 1);
TGT_HED_cmd                             = zeros(Sim_length, 1);
TGT_FPA_cmd                             = zeros(Sim_length, 1);
TGT_vel_NED                             = zeros(Sim_length, 3);
TGT_pos_NED                             = zeros(Sim_length, 3);

MSL_Euler                               = zeros(Sim_length, 3);
MSL_HED                                 = zeros(Sim_length, 1);
MSL_FPA                                 = zeros(Sim_length, 1);
MSL_PQR                                 = zeros(Sim_length, 3);
MSL_PQR_rate                            = zeros(Sim_length, 3);
MSL_Euler_rate                          = zeros(Sim_length, 3);
MSL_Euler_cmd                           = zeros(Sim_length, 3);
MSL_Torque                              = zeros(Sim_length, 3);
MSL_pos_NED                             = zeros(Sim_length, 3);
MSL_vel_NED                             = zeros(Sim_length, 3);
MSL_speed                               = zeros(Sim_length, 1);
MSL_vel_Body                            = zeros(Sim_length, 3);
MSL_vel_Wind                            = zeros(Sim_length, 3);
MSL_acc_cmd_NED                         = zeros(Sim_length, 3);
MSL_acc_cmd_Wind                        = zeros(Sim_length, 3);
MSL_acc_cmd_Body                        = zeros(Sim_length, 3);
MSL_acc_mod_Body                        = zeros(Sim_length, 3);
MSL_force_Body                          = zeros(Sim_length, 3);
MSL_acc_Body                            = zeros(Sim_length, 3);
MSL_acc_NED                             = zeros(Sim_length, 3);
MSL_acc_Wind                            = zeros(Sim_length, 3);
MSL_Thrust                              = zeros(Sim_length, 10);
MSL_FOV_Angle                           = zeros(Sim_length, 1);
MSL_FOV_InRange                         = zeros(Sim_length, 1);
MSL_Disp_Body_ax1                       = zeros(Sim_length, 3);
MSL_Disp_Body_ax2                       = zeros(Sim_length, 3);
MSL_Disp_Body_ax3                       = zeros(Sim_length, 3);
MSL_Disp_LOS_long                       = zeros(Sim_length, 3);
MSL_Disp_LOS_short                      = zeros(Sim_length, 3);
MSL_Disp_Vel_long                       = zeros(Sim_length, 3);
MSL_Disp_Vel_short                      = zeros(Sim_length, 3);
MSL_Disp_Body                           = zeros(3, 8, Sim_length);
MSL_Disp_Thrust                         = zeros(3, 20, Sim_length);
MSL_Disp_FOV_long                       = zeros(3, 8, Sim_length);
MSL_Disp_FOV_short                      = zeros(3, 8, Sim_length);
MSL_Disp_FOV_circle_long                = zeros(3, length(0 : 0.1 : 2.1*pi), Sim_length);
MSL_Disp_FOV_circle_short               = zeros(3, length(0 : 0.1 : 2.1*pi), Sim_length);

Geom_range_NED                          = zeros(Sim_length, 3);
Geom_range                              = zeros(Sim_length, 1);
Geom_LOS                                = zeros(Sim_length, 2);
Geom_LOS_rotation                       = zeros(Sim_length, 3);
Geom_vel_relative                       = zeros(Sim_length, 3);
Geom_speed_closing                      = zeros(Sim_length, 1);

%% Target Initial Values
TGT_TurnCounter(1)                      = 1;
TGT_TurnTime                            = [0 TGT_TurnTime Sim_Tf];
TGT_HEDRate                             = [0 TGT_HEDRate];
TGT_FPARate                             = [0 TGT_FPARate];
TGT_HED(1)                              = TGT_HED0*Sim_D2R;
TGT_HED_cmd(1)                          = TGT_HED0*Sim_D2R;
TGT_FPA(1)                              = TGT_FPA0*Sim_D2R;
TGT_FPA_cmd(1)                          = TGT_FPA0*Sim_D2R;
TGT_MTX_N2W                             = [cos(TGT_FPA(1))  0  -sin(TGT_FPA(1))
                                           0                1   0
                                           sin(TGT_FPA(1))  0   cos(TGT_FPA(1))]...
                                          *...
                                          [cos(TGT_HED(1))  sin(TGT_HED(1))  0
                                          -sin(TGT_HED(1))  cos(TGT_HED(1))  0
                                           0                0             1];
TGT_MTX_W2N                             = TGT_MTX_N2W';
TGT_vel_NED(1, 1 : 3)                   = [TGT_vel0 0 0]*TGT_MTX_W2N';
TGT_pos_NED(1, 1 : 3)                   = TGT_pos_NED0;

%% Missile Initial Values

% Missile Attitude
MSL_Euler(1, 1 : 3)                     = [0 MSL_FPA0 MSL_HED0]*Sim_D2R;
MSL_MTX_N2B                             = [1  0                     0
                                           0  cos(MSL_Euler(1, 1))  sin(MSL_Euler(1, 1))
                                           0 -sin(MSL_Euler(1, 1))  cos(MSL_Euler(1, 1))]...
                                          *...
                                          [cos(MSL_Euler(1, 2))  0  -sin(MSL_Euler(1, 2))
                                           0                     1   0
                                           sin(MSL_Euler(1, 2))  0   cos(MSL_Euler(1, 2))]...
                                          *...
                                          [cos(MSL_Euler(1, 3))  sin(MSL_Euler(1, 3))  0
                                          -sin(MSL_Euler(1, 3))  cos(MSL_Euler(1, 3))  0
                                           0                     0                     1];
MSL_MTX_B2N                             = MSL_MTX_N2B';

% Missile Flight Path
MSL_HED(1)                              = MSL_Euler(1, 3);
MSL_FPA(1)                              = MSL_Euler(1, 2);
MSL_MTX_N2W                             = [cos(MSL_FPA(1))  0 -sin(MSL_FPA(1))
                                           0             1  0
                                           sin(MSL_FPA(1))  0  cos(MSL_FPA(1))]...
                                          *...
                                          [cos(MSL_HED(1))  sin(MSL_HED(1))  0
                                          -sin(MSL_HED(1))  cos(MSL_HED(1))  0
                                           0             0             1];
MSL_MTX_W2N                             = MSL_MTX_N2W';

% Missile Line of Sight
Geom_range_NED(1, 1 : 3)                = TGT_pos_NED0 - MSL_pos_NED0;
Geom_range(1)                           = norm(Geom_range_NED(1, 1 : 3));
Geom_LOS_vert                           = -atan2(Geom_range_NED(1, 3), norm(Geom_range_NED(1, 1 : 2)));
Geom_LOS_horz                           = atan2(Geom_range_NED(1, 2), Geom_range_NED(1, 1));
Geom_LOS(1, 1 : 2)                      = [Geom_LOS_vert Geom_LOS_horz];
MSL_MTX_N2L                             = [cos(Geom_LOS(1, 1))  0 -sin(Geom_LOS(1, 1))
                                           0                    1  0
                                           sin(Geom_LOS(1, 1))  0  cos(Geom_LOS(1, 1))]...
                                          *...
                                          [cos(Geom_LOS(1, 2))  sin(Geom_LOS(1, 2))  0
                                          -sin(Geom_LOS(1, 2))  cos(Geom_LOS(1, 2))  0
                                           0                    0                    1];
MSL_MTX_L2N                             = MSL_MTX_N2L';

% Missile Torque, Eq 3.16
MSL_PQR(1, 1 : 3)                       = zeros(1, 3);
MSL_Euler_rate(1, 1 : 3)                = MSL_PQR(1, 1 : 3)...
                                          *...
                                          [1  sin(MSL_Euler(1, 1))*tan(MSL_Euler(1, 2))  cos(MSL_Euler(1, 1))*tan(MSL_Euler(1, 2))
                                           0  cos(MSL_Euler(1, 1))                   -sin(MSL_Euler(1, 1))
                                           0  sin(MSL_Euler(1, 1))/cos(MSL_Euler(1, 2))  cos(MSL_Euler(1, 1))/cos(MSL_Euler(1, 2))]';
MSL_Euler_cmd(1, 1 : 3)                 = [0 Geom_LOS(1, 1) Geom_LOS(1, 2)];
MSL_Torque(1, 1 : 3)                    = MSL_Euler_kp.*sin(MSL_Euler_cmd(1, 1 : 3) - MSL_Euler(1, 1 : 3)) - MSL_Euler_kv.*MSL_Euler_rate(1, 1 : 3);

% Missile PQR with Moment of Inertia, Eq 3.17
MSL_Moment                              = MSL_Moment_Jxyz(1)*MSL_Moment_Jxyz(3);
MSL_Moment2                             = (MSL_Moment_Jxyz(3) - MSL_Moment_Jxyz(2))/MSL_Moment_Jxyz(1);
MSL_Moment3                             = 1/MSL_Moment_Jxyz(1);
MSL_Moment5                             = (MSL_Moment_Jxyz(3) - MSL_Moment_Jxyz(1))/MSL_Moment_Jxyz(2);
MSL_Moment7                             = (MSL_Moment_Jxyz(1) - MSL_Moment_Jxyz(2))/MSL_Moment_Jxyz(3);
MSL_Moment8                             = 1/MSL_Moment_Jxyz(3);
MSL_PQR_rate(1, 1 : 3)                  = [MSL_Moment3*MSL_Torque(1, 1) MSL_Torque(1, 2)/MSL_Moment_Jxyz(2) MSL_Moment8*MSL_Torque(1, 3)];

% Missile Guidance Law, Eq 3.15
MSL_pos_NED(1, 1 : 3)                   = MSL_pos_NED0;
MSL_vel_NED(1, 1 : 3)                   = [MSL_vel0 0 0]*MSL_MTX_W2N';
MSL_speed(1)                            = norm(MSL_vel_NED(1, 1 : 3));
MSL_vel_Body(1, 1 : 3)                  = MSL_vel_NED(1, 1 : 3)*MSL_MTX_N2B';
MSL_vel_Wind(1, 1 : 3)                  = MSL_vel_NED(1, 1 : 3)*MSL_MTX_N2W';
Geom_vel_relative(1, 1 : 3)             = TGT_vel_NED(1, 1 : 3) - MSL_vel_NED(1, 1 : 3);
Geom_speed_closing(1)                   = norm(Geom_vel_relative(1, 1 : 3));
Geom_LOS_rotation(1, 1 : 3)             = cross(Geom_range_NED(1, 1 : 3), Geom_vel_relative(1, 1 : 3))/Geom_range(1)^2;
MSL_acc_cmd_NED(1, 1 : 3)               = -MSL_nav_gain*norm(Geom_vel_relative(1, 1 : 3))/MSL_speed(1)*cross(MSL_vel_NED(1, 1 : 3), Geom_LOS_rotation(1, 1 : 3));
MSL_acc_cmd_Wind(1, 1 : 3)              = MSL_acc_cmd_NED(1, 1 : 3)*MSL_MTX_N2W';
MSL_acc_cmd_Body(1, 1 : 3)              = MSL_acc_cmd_NED(1, 1 : 3)*MSL_MTX_N2B';
MSL_acc_mod_Body(1, 1 : 3)              = [0 MSL_acc_cmd_Body(1, 2 : 3)/cos(atan2(MSL_acc_cmd_Body(1, 1), norm(MSL_acc_cmd_Body(1, 2 : 3))))^2];

% Missile Acceleration w/ Torque, Eq 3.15
MSL_force_Body(1, 1 : 3)                = MSL_Mass*MSL_acc_mod_Body(1, 1 : 3);
MSL_acc_Body(1, 1 : 3)                  = [MSL_PQR(1, 3)*MSL_vel_Body(1, 2) - MSL_PQR(1, 2)*MSL_vel_Body(1, 3)
                                           MSL_PQR(1, 1)*MSL_vel_Body(1, 3) - MSL_PQR(1, 3)*MSL_vel_Body(1, 1)
                                           MSL_PQR(1, 2)*MSL_vel_Body(1, 1) - MSL_PQR(1, 1)*MSL_vel_Body(1, 2)]'...
                                          +...
                                          MSL_force_Body(1, 1 : 3)/MSL_Mass...
                                          +...
                                          [0 0 Sim_G]*MSL_MTX_N2B';
MSL_acc_NED(1, 1 : 3)                   = MSL_acc_Body(1, 1 : 3)*MSL_MTX_B2N';
MSL_acc_Wind(1, 1 : 3)                  = MSL_acc_NED(1, 1 : 3)*MSL_MTX_N2W';

% DACS Thrust
MSL_Thrust_cmd                          = [MSL_force_Body(1, 2 : 3) MSL_Torque(1, 1 : 3)]';
MSL_Thrust_MTX                          = [0  1  0                        1                        1
                                           1  0  1                        0                        0
                                           0  0  0                        Disp_Body_Size(3)/4 -Disp_Body_Size(3)/4
                                           0  0  Disp_Body_Size(1)/2  0                        0
                                           0  0  0                       -Disp_Body_Size(1)/2 -Disp_Body_Size(1)/2];
MSL_Thrust_temp                         = (MSL_Thrust_MTX\MSL_Thrust_cmd)';
MSL_MTX_Thrustmap                       = [1  0  0  0  0
                                           0 -1  0  0  0
                                          -1  0  0  0  0
                                           0  1  0  0  0
                                           0  0  1  0  0
                                           0  0  0 -1  0
                                           0  0  0  0 -1
                                           0  0 -1  0  0
                                           0  0  0  0  1
                                           0  0  0  1  0]';
MSL_Thrust(1, 1 : 10)                   = max(MSL_Thrust_temp*MSL_MTX_Thrustmap, 0);

% Missile Seeker
MSL_FOV_Body_ax                         = [1 0 0]*MSL_MTX_B2N';
MSL_FOV_LOS_ax                          = [1 0 0]*MSL_MTX_L2N';
MSL_FOV_Angle(1)                        = acos(dot(MSL_FOV_Body_ax, MSL_FOV_LOS_ax));
MSL_FOV_InRange(1)                      = max(sign(MSL_FOV/2*Sim_D2R - MSL_FOV_Angle(1)), 0);

%% Vector Visualization

% Body Axis
MSL_Disp_Body_ax1(1, :)                 = [Disp_Body_axlength 0 0]*MSL_MTX_B2N';
MSL_Disp_Body_ax2(1, :)                 = [0 Disp_Body_axlength 0]*MSL_MTX_B2N';
MSL_Disp_Body_ax3(1, :)                 = [0 0 Disp_Body_axlength]*MSL_MTX_B2N';

% LOS Axis
MSL_Disp_LOS_long(1, :)                 = [Geom_range(1) 0 0]*MSL_MTX_L2N';
MSL_Disp_LOS_short(1, :)                = MSL_Disp_LOS_long(1, :)/Geom_range(1)*Disp_Body_axlength*2;

% Velocity Axis
MSL_Disp_Vel_long(1, :)                 = [MSL_speed(1) 0 0]*MSL_MTX_W2N';
MSL_Disp_Vel_short(1, :)                = MSL_Disp_Vel_long(1, :)/MSL_speed(1)*Disp_Body_axlength*2;

% Body Shape
MSL_Disp_Body_temp(:, 1)                = [ Disp_Body_Size(1)/2; -Disp_Body_Size(2)/2;  Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 2)                = [ Disp_Body_Size(1)/2;  Disp_Body_Size(2)/2;  Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 3)                = [ Disp_Body_Size(1)/2; -Disp_Body_Size(2)/2; -Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 4)                = [ Disp_Body_Size(1)/2;  Disp_Body_Size(2)/2; -Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 5)                = [-Disp_Body_Size(1)/2; -Disp_Body_Size(2)/2;  Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 6)                = [-Disp_Body_Size(1)/2;  Disp_Body_Size(2)/2;  Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 7)                = [-Disp_Body_Size(1)/2; -Disp_Body_Size(2)/2; -Disp_Body_Size(3)/2];
MSL_Disp_Body_temp(:, 8)                = [-Disp_Body_Size(1)/2;  Disp_Body_Size(2)/2; -Disp_Body_Size(3)/2];
MSL_Disp_Body(:, :, 1)                  = MSL_MTX_B2N*MSL_Disp_Body_temp + MSL_pos_NED(1, 1 : 3)';

% Thruster Position
MSL_Disp_Thrust_temp(:, 1)              = [ 0                        0                       -Disp_Body_Size(3)/2];
MSL_Disp_Thrust_temp(:, 2)              = [ 0                        Disp_Body_Size(2)/2  0                      ];
MSL_Disp_Thrust_temp(:, 3)              = [ 0                        0                        Disp_Body_Size(3)/2];
MSL_Disp_Thrust_temp(:, 4)              = [ 0                       -Disp_Body_Size(2)/2  0                      ];
MSL_Disp_Thrust_temp(:, 5)              = [-Disp_Body_Size(1)/2  0                       -Disp_Body_Size(3)/2];
MSL_Disp_Thrust_temp(:, 6)              = [-Disp_Body_Size(1)/2  Disp_Body_Size(2)/2 -Disp_Body_Size(3)/4];
MSL_Disp_Thrust_temp(:, 7)              = [-Disp_Body_Size(1)/2  Disp_Body_Size(2)/2  Disp_Body_Size(3)/4];
MSL_Disp_Thrust_temp(:, 8)              = [-Disp_Body_Size(1)/2  0                        Disp_Body_Size(3)/2];
MSL_Disp_Thrust_temp(:, 9)              = [-Disp_Body_Size(1)/2 -Disp_Body_Size(2)/2  Disp_Body_Size(3)/4];
MSL_Disp_Thrust_temp(:, 10)             = [-Disp_Body_Size(1)/2 -Disp_Body_Size(2)/2 -Disp_Body_Size(3)/4];

% Thrust Vector
MSL_Disp_Thrust_temp(:, 11)             = MSL_Disp_Thrust_temp(:,  1) + [0  0                 -MSL_Thrust(1, 1)]'*Disp_Thrust_DCS_Scale;
MSL_Disp_Thrust_temp(:, 12)             = MSL_Disp_Thrust_temp(:,  2) + [0  MSL_Thrust(1, 2)   0               ]'*Disp_Thrust_DCS_Scale;
MSL_Disp_Thrust_temp(:, 13)             = MSL_Disp_Thrust_temp(:,  3) + [0  0                  MSL_Thrust(1, 3)]'*Disp_Thrust_DCS_Scale;
MSL_Disp_Thrust_temp(:, 14)             = MSL_Disp_Thrust_temp(:,  4) + [0 -MSL_Thrust(1, 4)   0               ]'*Disp_Thrust_DCS_Scale;
MSL_Disp_Thrust_temp(:, 15)             = MSL_Disp_Thrust_temp(:,  5) + [0  0                 -MSL_Thrust(1, 5)]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust_temp(:, 16)             = MSL_Disp_Thrust_temp(:,  6) + [0  MSL_Thrust(1, 6)   0               ]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust_temp(:, 17)             = MSL_Disp_Thrust_temp(:,  7) + [0  MSL_Thrust(1, 7)   0               ]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust_temp(:, 18)             = MSL_Disp_Thrust_temp(:,  8) + [0  0                  MSL_Thrust(1, 8)]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust_temp(:, 19)             = MSL_Disp_Thrust_temp(:,  9) + [0 -MSL_Thrust(1, 9)   0               ]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust_temp(:, 20)             = MSL_Disp_Thrust_temp(:, 10) + [0 -MSL_Thrust(1, 10)  0               ]'*Disp_Thrust_ACS_Scale;
MSL_Disp_Thrust(:, :, 1)                = MSL_MTX_B2N*MSL_Disp_Thrust_temp + MSL_pos_NED(1, 1 : 3)';

% Seeker FOV
MSL_Disp_FOV_temp(:, 1)                 = [cos(MSL_FOV/2*Sim_D2R)   0                               -sin(MSL_FOV/2*Sim_D2R)];
MSL_Disp_FOV_temp(:, 2)                 = [MSL_Disp_FOV_temp(1, 1) -MSL_Disp_FOV_temp(3, 1)/sqrt(2) -MSL_Disp_FOV_temp(3, 1)/sqrt(2)];
MSL_Disp_FOV_temp(:, 3)                 = [MSL_Disp_FOV_temp(1, 1) -MSL_Disp_FOV_temp(3, 1)          0];
MSL_Disp_FOV_temp(:, 4)                 = [MSL_Disp_FOV_temp(1, 1) -MSL_Disp_FOV_temp(3, 1)/sqrt(2)  MSL_Disp_FOV_temp(3, 1)/sqrt(2)];
MSL_Disp_FOV_temp(:, 5)                 = [MSL_Disp_FOV_temp(1, 1)  0                               -MSL_Disp_FOV_temp(3, 1)];
MSL_Disp_FOV_temp(:, 6)                 = [MSL_Disp_FOV_temp(1, 1)  MSL_Disp_FOV_temp(3, 1)/sqrt(2)  MSL_Disp_FOV_temp(3, 1)/sqrt(2)];
MSL_Disp_FOV_temp(:, 7)                 = [MSL_Disp_FOV_temp(1, 1)  MSL_Disp_FOV_temp(3, 1)          0];
MSL_Disp_FOV_temp(:, 8)                 = [MSL_Disp_FOV_temp(1, 1)  MSL_Disp_FOV_temp(3, 1)/sqrt(2) -MSL_Disp_FOV_temp(3, 1)/sqrt(2)];
MSL_Disp_FOV_long(:, :, 1)              = MSL_MTX_B2N*MSL_Disp_FOV_temp*Geom_range(1) + MSL_pos_NED(1, 1 : 3)';
MSL_Disp_FOV_short(:, :, 1)             = (MSL_Disp_FOV_long(:, :, 1) - MSL_pos_NED(1, 1 : 3)')/Geom_range(1)*Disp_Body_axlength*2 + MSL_pos_NED(1, 1 : 3)';
MSL_Disp_FOV_circle_temp(2, :)          = MSL_Disp_FOV_temp(3, 1)*cos(0 : 0.1 : 2.1*pi);
MSL_Disp_FOV_circle_temp(3, :)          = MSL_Disp_FOV_temp(3, 1)*sin(0 : 0.1 : 2.1*pi);
MSL_Disp_FOV_circle_temp(1, :)          = MSL_Disp_FOV_temp(1, 1);
MSL_Disp_FOV_circle_long(:, :, 1)       = MSL_MTX_B2N*MSL_Disp_FOV_circle_temp*Geom_range(1) + MSL_pos_NED(1, 1 : 3)';
MSL_Disp_FOV_circle_short(:, :, 1)      = (MSL_Disp_FOV_circle_long(:, :, 1) - MSL_pos_NED(1, 1 : 3)')/Geom_range(1)*Disp_Body_axlength*2 + MSL_pos_NED(1, 1 : 3)';

%% Loop Begins

for i = 2 : length(Sim_time)
    %% Target Trajectory
    if Sim_time(i) >= TGT_TurnTime(TGT_TurnCounter) && Sim_time(i) < TGT_TurnTime(TGT_TurnCounter + 1)
        TGT_HED_cmd(i, 1)               = TGT_HED_cmd(i - 1, 1) + Sim_dt*TGT_HEDRate(TGT_TurnCounter)*Sim_D2R;
        TGT_FPA_cmd(i, 1)               = TGT_FPA_cmd(i - 1, 1) + Sim_dt*TGT_FPARate(TGT_TurnCounter)*Sim_D2R;
    elseif Sim_time(i) >= TGT_TurnTime(TGT_TurnCounter + 1)
        TGT_HED_cmd(i, 1)               = TGT_HED_cmd(i - 1, 1);
        TGT_FPA_cmd(i, 1)               = TGT_FPA_cmd(i - 1, 1);
        TGT_TurnCounter                 = TGT_TurnCounter + 1;
    end
    TGT_HED(i, 1)                       = TGT_HED(i - 1, 1) + Sim_dt*TGT_BW*(TGT_HED_cmd(i, 1) - TGT_HED(i - 1, 1));
    TGT_FPA(i, 1)                       = TGT_FPA(i - 1, 1) + Sim_dt*TGT_BW*(TGT_FPA_cmd(i, 1) - TGT_FPA(i - 1, 1));
    TGT_MTX_N2W                         = [cos(TGT_FPA(i, 1))  0  -sin(TGT_FPA(i, 1))
                                           0                   1   0
                                           sin(TGT_FPA(i, 1))  0   cos(TGT_FPA(i, 1))]...
                                          *...
                                          [cos(TGT_HED(i, 1))  sin(TGT_HED(i, 1))  0
                                          -sin(TGT_HED(i, 1))  cos(TGT_HED(i, 1))  0
                                           0                   0                   1];
    TGT_MTX_W2N = TGT_MTX_N2W';
    TGT_vel_NED(i, 1 : 3)                = [TGT_vel0 0 0]*TGT_MTX_W2N';
    TGT_pos_NED(i, 1 : 3)                = TGT_pos_NED(i - 1, 1 : 3) + Sim_dt*TGT_vel_NED(i, 1 : 3);
    
    %% Numerical Integration
    if i == 2
        MSL_pos_NED(2, 1 : 3)            = 1.5*Sim_dt*MSL_vel_NED(1, 1 : 3) + MSL_pos_NED(1, 1 : 3);
        MSL_vel_Body(2, 1 : 3)           = 1.5*Sim_dt*MSL_acc_Body(1, 1 : 3) + MSL_vel_Body(1, 1 : 3);
        MSL_Euler(2, 1 : 3)              = 1.5*Sim_dt*MSL_Euler_rate(1, 1 : 3) + MSL_Euler(1, 1 : 3);
        MSL_PQR(2, 1 : 3)                = 1.5*Sim_dt*MSL_PQR_rate(1, 1 : 3) + MSL_PQR(1, 1 : 3);
    else
        MSL_pos_NED(i, 1 : 3)            = Sim_dt/2*(3*MSL_vel_NED(i - 1, 1 : 3) - MSL_vel_NED(i - 2, 1 : 3)) + MSL_pos_NED(i - 1, 1 : 3);
        MSL_vel_Body(i, 1 : 3)           = Sim_dt/2*(3*MSL_acc_Body(i - 1, 1 : 3) - MSL_acc_Body(i - 2, 1 : 3)) + MSL_vel_Body(i - 1, 1 : 3);
        MSL_Euler(i, 1 : 3)              = Sim_dt/2*(3*MSL_Euler_rate(i - 1, 1 : 3) - MSL_Euler_rate(i - 2, 1 : 3)) + MSL_Euler(i - 1, 1 : 3);
        MSL_PQR(i, 1 : 3)                = Sim_dt/2*(3*MSL_PQR_rate(i - 1, 1 : 3) - MSL_PQR_rate(i - 2, 1 : 3)) + MSL_PQR(i - 1, 1 : 3);
    end
    
    %% Equation 3.14
    MSL_MTX_N2B                          = [1  0                     0
                                            0  cos(MSL_Euler(i, 1))  sin(MSL_Euler(i, 1))
                                            0 -sin(MSL_Euler(i, 1))  cos(MSL_Euler(i, 1))]...
                                           *...
                                           [cos(MSL_Euler(i, 2))  0 -sin(MSL_Euler(i, 2))
                                            0                     1  0
                                            sin(MSL_Euler(i, 2))  0  cos(MSL_Euler(i, 2))]...
                                           *...
                                           [cos(MSL_Euler(i, 3))  sin(MSL_Euler(i, 3))  0
                                           -sin(MSL_Euler(i, 3))  cos(MSL_Euler(i, 3))  0
                                            0                     0                     1];
    MSL_MTX_B2N                          = MSL_MTX_N2B';                                    
    MSL_vel_NED(i, 1 : 3)                = MSL_vel_Body(i, 1 : 3)*MSL_MTX_B2N';
    MSL_speed(i, 1)                      = norm(MSL_vel_NED(i, 1 : 3));
    MSL_HED(i, 1)                        = atan2(MSL_vel_NED(i, 2), MSL_vel_NED(i, 1));
    MSL_FPA(i, 1)                        = -atan2(MSL_vel_NED(i, 3), norm(MSL_vel_NED(i, 1 : 2)));
    MSL_MTX_N2W                          = [cos(MSL_FPA(i, 1))  0 -sin(MSL_FPA(i, 1))
                                            0                   1  0
                                            sin(MSL_FPA(i, 1))  0  cos(MSL_FPA(i, 1))]...
                                           *...
                                           [cos(MSL_HED(i, 1))  sin(MSL_HED(i, 1))  0
                                           -sin(MSL_HED(i, 1))  cos(MSL_HED(i, 1))  0
                                            0                   0                   1];
    MSL_MTX_W2N                          = MSL_MTX_N2W';
    MSL_vel_Wind(i, 1 : 3)               = MSL_vel_NED(i, 1 : 3)*MSL_MTX_W2N';
    
    %% Equation 3.15 & Acceleration Command
    Geom_range_NED(i, 1 : 3)             = TGT_pos_NED(i, 1 : 3) - MSL_pos_NED(i, 1 : 3);
    Geom_range(i, 1)                     = norm(Geom_range_NED(i, 1 : 3));
    Geom_LOS_vert                        = -atan2(Geom_range_NED(i, 3), norm(Geom_range_NED(i, 1 : 2)));
    Geom_LOS_horz                        = atan2(Geom_range_NED(i, 2), Geom_range_NED(i, 1));
    Geom_LOS(i, 1 : 2)                   = [Geom_LOS_vert Geom_LOS_horz];
    MSL_MTX_N2L                          = [cos(Geom_LOS(i, 1)) 0 -sin(Geom_LOS(i, 1))
                                            0                   1  0
                                            sin(Geom_LOS(i, 1)) 0  cos(Geom_LOS(i, 1))]...
                                           *...
                                           [cos(Geom_LOS(i, 2))  sin(Geom_LOS(i, 2))  0
                                           -sin(Geom_LOS(i, 2))  cos(Geom_LOS(i, 2))  0
                                            0                    0                    1];
    MSL_MTX_L2N = MSL_MTX_N2L';
    Geom_vel_relative(i, 1 : 3)          = TGT_vel_NED(i, 1 : 3) - MSL_vel_NED(i, 1 : 3);
    Geom_speed_closing(i, 1)             = norm(Geom_vel_relative(i, 1 : 3));
    Geom_LOS_rotation(i, 1 : 3)          = cross(Geom_range_NED(i, 1 : 3), Geom_vel_relative(i, 1 : 3))/Geom_range(i, 1)^2;
    MSL_acc_cmd_NED(i, 1 : 3)            = -MSL_nav_gain*norm(Geom_vel_relative(i, 1 : 3))/MSL_speed(i, 1)*cross(MSL_vel_NED(i, 1 : 3), Geom_LOS_rotation(i, 1 : 3));
    MSL_acc_cmd_Wind(i, 1 : 3)           = MSL_acc_cmd_NED(i, 1 : 3)*MSL_MTX_N2W';
    MSL_acc_cmd_Body(i, 1 : 3)           = MSL_acc_cmd_NED(i, 1 : 3)*MSL_MTX_N2B';
    MSL_acc_mod_Body(i, 1 : 3)           = [0 MSL_acc_cmd_Body(i, 2 : 3)/cos(atan2(MSL_acc_cmd_Body(i, 1), norm(MSL_acc_cmd_Body(i, 2 : 3))))^2];
    MSL_force_Body(i, 1 : 3)             = MSL_Mass*(MSL_acc_mod_Body(i, 1 : 3));
    MSL_acc_Body(i, 1 : 3)               = [MSL_PQR(i, 3)*MSL_vel_Body(i, 2) - MSL_PQR(i, 2)*MSL_vel_Body(i, 3)
                                            MSL_PQR(i, 1)*MSL_vel_Body(i, 3) - MSL_PQR(i, 3)*MSL_vel_Body(i, 1)
                                            MSL_PQR(i, 2)*MSL_vel_Body(i, 1) - MSL_PQR(i, 1)*MSL_vel_Body(i, 2)]'...
                                           +...
                                            MSL_force_Body(i, 1 : 3)/MSL_Mass...
                                           +...
                                           [0 0 Sim_G]*MSL_MTX_N2B';
    MSL_acc_NED(i, 1 : 3)                = MSL_acc_Body(i, 1 : 3)*MSL_MTX_B2N';
    MSL_acc_Wind(i, 1 : 3)               = MSL_acc_NED(i, 1 : 3)*MSL_MTX_N2W';
    
    %% Equation 3.16
    MSL_Euler_rate(i, 1 : 3)             = MSL_PQR(i, 1 : 3)...
                                           *...
                                           [1  sin(MSL_Euler(i, 1))*tan(MSL_Euler(i, 2))  cos(MSL_Euler(i, 1))*tan(MSL_Euler(i, 2))
                                            0  cos(MSL_Euler(i, 1))                      -sin(MSL_Euler(i, 1))
                                            0  sin(MSL_Euler(i, 1))/cos(MSL_Euler(i, 2))  cos(MSL_Euler(i, 1))/cos(MSL_Euler(i, 2))]';
    
    %% Equation 3.17
    MSL_Euler_cmd(i, 1 : 3)              = [0 Geom_LOS(i, 1) Geom_LOS(i, 2)];
    MSL_Torque(i, 1 : 3)                 = MSL_Euler_kp.*(sin(MSL_Euler_cmd(i, 1 : 3) - MSL_Euler(i, 1 : 3))) - MSL_Euler_kv.*MSL_Euler_rate(i, 1 : 3);
    MSL_PQR_rate(i, 1 : 3)               = [MSL_Moment3*MSL_Torque(i, 1) MSL_Torque(i, 2)/MSL_Moment_Jxyz(2) MSL_Moment8*MSL_Torque(i, 3)];
    
    %% DACS Thrust
    MSL_Thrust_cmd                       = [MSL_force_Body(i, 2 : 3) MSL_Torque(i, 1 : 3)]';
    MSL_Thrust_temp                      = (MSL_Thrust_MTX\MSL_Thrust_cmd)';
    MSL_Thrust(i, 1 : 10)                = max(MSL_Thrust_temp*MSL_MTX_Thrustmap, 0);
    
    %% Seeker FOV
    MSL_FOV_Body_ax                      = [1 0 0]*MSL_MTX_B2N';
    MSL_FOV_LOS_ax                       = [1 0 0]*MSL_MTX_L2N';
    MSL_FOV_Angle(i, 1)                  = acos(dot(MSL_FOV_Body_ax, MSL_FOV_LOS_ax));
    MSL_FOV_InRange(i, 1)                = max(sign(MSL_FOV/2*Sim_D2R - MSL_FOV_Angle(i, 1)), 0);
    
    %% Vector Visualization
    MSL_Disp_Body_ax1(i, 1 : 3)          = [Disp_Body_axlength 0 0]*MSL_MTX_B2N';
    MSL_Disp_Body_ax2(i, 1 : 3)          = [0 Disp_Body_axlength 0]*MSL_MTX_B2N';
    MSL_Disp_Body_ax3(i, 1 : 3)          = [0 0 Disp_Body_axlength]*MSL_MTX_B2N';
    
    MSL_Disp_LOS_long(i, 1 : 3)          = [Geom_range(i, 1) 0 0]*MSL_MTX_L2N';
    MSL_Disp_LOS_short(i, 1 : 3)         = MSL_Disp_LOS_long(i, 1 : 3)/Geom_range(i, 1)*Disp_Body_axlength*2;
    
    MSL_Disp_Vel_long(i, 1 : 3)          = [MSL_speed(i, 1) 0 0]*MSL_MTX_W2N';
    MSL_Disp_Vel_short(i, 1 : 3)         = MSL_Disp_Vel_long(i, 1 : 3)/MSL_speed(i, 1)*Disp_Body_axlength*2;
    
    MSL_Disp_Body(:, :, i)               = MSL_MTX_B2N*MSL_Disp_Body_temp + MSL_pos_NED(i, 1 : 3)';
    
    MSL_Disp_Thrust_temp(:, 11)         = MSL_Disp_Thrust_temp(:,  1) + [0  0                 -MSL_Thrust(i, 1)]'*Disp_Thrust_DCS_Scale;
    MSL_Disp_Thrust_temp(:, 12)         = MSL_Disp_Thrust_temp(:,  2) + [0  MSL_Thrust(i, 2)   0               ]'*Disp_Thrust_DCS_Scale;
    MSL_Disp_Thrust_temp(:, 13)         = MSL_Disp_Thrust_temp(:,  3) + [0  0                  MSL_Thrust(i, 3)]'*Disp_Thrust_DCS_Scale;
    MSL_Disp_Thrust_temp(:, 14)         = MSL_Disp_Thrust_temp(:,  4) + [0 -MSL_Thrust(i, 4)   0               ]'*Disp_Thrust_DCS_Scale;
    MSL_Disp_Thrust_temp(:, 15)         = MSL_Disp_Thrust_temp(:,  5) + [0  0                 -MSL_Thrust(i, 5)]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust_temp(:, 16)         = MSL_Disp_Thrust_temp(:,  6) + [0  MSL_Thrust(i, 6)   0               ]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust_temp(:, 17)         = MSL_Disp_Thrust_temp(:,  7) + [0  MSL_Thrust(i, 7)   0               ]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust_temp(:, 18)         = MSL_Disp_Thrust_temp(:,  8) + [0  0                  MSL_Thrust(i, 8)]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust_temp(:, 19)         = MSL_Disp_Thrust_temp(:,  9) + [0 -MSL_Thrust(i, 9)   0               ]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust_temp(:, 20)         = MSL_Disp_Thrust_temp(:, 10) + [0 -MSL_Thrust(i, 10)  0               ]'*Disp_Thrust_ACS_Scale;
    MSL_Disp_Thrust(:, :, i)            = MSL_MTX_B2N*MSL_Disp_Thrust_temp + MSL_pos_NED(i, 1 : 3)';
    
    MSL_Disp_FOV_long(:, :, i)          = MSL_MTX_B2N*MSL_Disp_FOV_temp*Geom_range(i, 1) + MSL_pos_NED(i, 1 : 3)';
    MSL_Disp_FOV_short(:, :, i)         = (MSL_Disp_FOV_long(:, :, i) - MSL_pos_NED(i, 1 : 3)')/Geom_range(i, 1)*Disp_Body_axlength*2 + MSL_pos_NED(i, 1 : 3)';
    MSL_Disp_FOV_circle_long(:, :, i)   = MSL_MTX_B2N*MSL_Disp_FOV_circle_temp*Geom_range(i, 1) + MSL_pos_NED(i, 1 : 3)';
    MSL_Disp_FOV_circle_short(:, :, i)  = (MSL_Disp_FOV_circle_long(:, :, i) - MSL_pos_NED(i, 1 : 3)')/Geom_range(i, 1)*Disp_Body_axlength*2 + MSL_pos_NED(i, 1 : 3)';
    
    %% Save and Terminate
    if Geom_range(i, 1) - Geom_range(i - 1, 1) > 0
        Result.Time                     = Sim_time(1 : i)';
        Result.Target_Traj              = TGT_pos_NED(1 : i, :);
        Result.Target_Vel               = TGT_vel_NED(1 : i, :);
        Result.Target_Heading           = TGT_HED(1 : i, :);
        Result.Target_FlightPath        = TGT_FPA(1 : i, :);
        Result.Target_Heading_cmd       = TGT_HED_cmd(1 : i, :);
        Result.Target_FlightPath_cmd    = TGT_FPA_cmd(1 : i, :);
        
        Result.Missile_Euler            = MSL_Euler(1 : i, :);
        Result.Missile_Heading          = MSL_HED(1 : i, :);
        Result.Missile_FPAngle          = MSL_FPA(1 : i, :);
        Result.Missile_Euler_PQR        = MSL_PQR(1 : i, :);
        Result.Missile_Euler_PQR_Rate   = MSL_PQR_rate(1 : i, :);
        Result.Missile_Euler_Rate       = MSL_Euler_rate(1 : i, :);
        Result.Missile_Euler_cmd        = MSL_Euler_cmd(1 : i, :);
        Result.Missile_Torque           = MSL_Torque(1 : i, :);
        Result.Missile_Traj             = MSL_pos_NED(1 : i, :);
        Result.Missile_Vel_NED          = MSL_vel_NED(1 : i, :);
        Result.Missile_Speed            = MSL_speed(1 : i, :);
        Result.Missile_Vel_Body         = MSL_vel_Body(1 : i, :);
        Result.Missile_Vel_Wind         = MSL_vel_Wind(1 : i, :);
        Result.Missile_Acc_Cmd_NED      = MSL_acc_cmd_NED(1 : i, :);
        Result.Missile_Acc_Cmd_Wind     = MSL_acc_cmd_Wind(1 : i, :);
        Result.Missile_Acc_Cmd_Body     = MSL_acc_cmd_Body(1 : i, :);
        Result.Missile_Acc_Cmd_Mod_Body = MSL_acc_mod_Body(1 : i, :);
        Result.Missile_Force_Body       = MSL_force_Body(1 : i, :);
        Result.Missile_Acc_Body         = MSL_acc_Body(1 : i, :);
        Result.Missile_Acc_NED          = MSL_acc_NED(1 : i, :);
        Result.Missile_Acc_Wind         = MSL_acc_Wind(1 : i, :);
        Result.Missile_Thrust           = MSL_Thrust(1 : i, :);
        Result.Missile_FOV_Angle        = MSL_FOV_Angle(1 : i, :);
        Result.Missile_FOV_InRange      = MSL_FOV_InRange(1 : i, :);
        
        Result.Geometry_Range_NED       = Geom_range_NED(1 : i, :);
        Result.Geometry_Range           = Geom_range(1 : i, :);
        Result.Geometry_LOS             = Geom_LOS(1 : i, :);
        Result.Geometry_LOS_Rotation    = Geom_LOS_rotation(1 : i, :);
        Result.Geometry_Vel_Relative    = Geom_vel_relative(1 : i, :);
        Result.Geometry_Speed_Closing   = Geom_speed_closing(1 : i, :);
        
        Result.Display_Body_ax1         = MSL_Disp_Body_ax1(1 : i, :);
        Result.Display_Body_ax2         = MSL_Disp_Body_ax2(1 : i, :);
        Result.Display_Body_ax3         = MSL_Disp_Body_ax3(1 : i, :);
        Result.Display_LOS_long         = MSL_Disp_LOS_long(1 : i, :);
        Result.Display_LOS_short        = MSL_Disp_LOS_short(1 : i, :);
        Result.Display_Vel_long         = MSL_Disp_Vel_long(1 : i, :);
        Result.Display_Vel_short        = MSL_Disp_Vel_short(1 : i, :);
        Result.Display_Body             = MSL_Disp_Body(:, :, 1 : i);
        Result.Display_Thrust           = MSL_Disp_Thrust(:, :, 1 : i);
        Result.Display_FOV_long         = MSL_Disp_FOV_long(:, :, 1 : i);
        Result.Display_FOV_short        = MSL_Disp_FOV_short(:, :, 1 : i);
        Result.Display_FOV_circle_long  = MSL_Disp_FOV_circle_long(:, :, 1 : i);
        Result.Display_FOV_circle_short = MSL_Disp_FOV_circle_short(:, :, 1 : i);
        
        disp(' '); disp(['Flight Time : ' num2str(max(Result.Time)) ' [sec]']);
        disp(['Miss Distance : ' num2str(min(Result.Geometry_Range)) ' [m]']);
        disp(['Tracking Score : ' num2str(round(sum(Result.Missile_FOV_InRange)/i*100)) '/100']);
        disp(' '); disp('=============================================='); disp(' ');
        break;
    end

end
