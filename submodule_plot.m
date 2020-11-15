close all;

figure('name', 'Scenario Trajectory');
plot3(Result.Missile_Traj(1 : end, 2), Result.Missile_Traj(1 : end, 1), -Result.Missile_Traj(1 : end, 3)); hold on;
plot3(Result.Target_Traj(1 : end, 2), Result.Target_Traj(1 : end, 1), -Result.Target_Traj(1 : end, 3)); hold on;
plot3(Result.Missile_Traj(1, 2), Result.Missile_Traj(1, 1), -Result.Missile_Traj(1, 3), 'ok'); hold on;
plot3(Result.Missile_Traj(end, 2), Result.Missile_Traj(end, 1), -Result.Missile_Traj(end, 3), 'xr'); hold on;
plot3(Result.Target_Traj(1, 2), Result.Target_Traj(1, 1), -Result.Target_Traj(1, 3), 'ok'); hold on;
plot3(Result.Target_Traj(end, 2), Result.Target_Traj(end, 1), -Result.Target_Traj(end, 3), 'xr'); grid on;
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]'); axis equal;
legend('Missile', 'Target', 'Start', 'End');

figure('name', 'Target FPA, HED');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Target_FlightPath_cmd(1 : end - 5, 1)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Target_FlightPath(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('FPA [deg]'); legend('FPA cmd', 'FPA');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Target_Heading_cmd(1 : end - 5, 1)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Target_Heading(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('HED [deg]'); legend('HED cmd', 'HED');

figure('name', 'Line of Sight');
subplot(3, 1, 1);
plot(Result.Time(1 : end), Result.Geometry_Range(1 : end, 1)); grid on;
xlabel('time [sec]'); ylabel('Range [m]'); legend('Range');
subplot(3, 1, 2);
plot(Result.Time(1 : end - 5), Result.Geometry_LOS(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('LOS [deg]'); legend('LOS(Vertical)');
subplot(3, 1, 3);
plot(Result.Time(1 : end - 5), Result.Geometry_LOS(1 : end - 5, 2)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('LOS [deg]'); legend('LOS(Horizontal)');

figure('name', 'Speed, FPA, HED');
subplot(3, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Speed(1 : end - 5, 1)); grid on;
xlabel('time [sec]'); ylabel('Speed [m/s]'); legend('Speed');
subplot(3, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_FPAngle(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('FPA [deg]'); legend('FPA(Vertical)');
subplot(3, 1, 3);
plot(Result.Time(1 : end - 5), Result.Missile_Heading(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('HED [deg]'); legend('HED(Horizontal)');

figure('name', 'Acceleration Command');
subplot(3, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Wind(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Wind(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Wind(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Original(VV axis 1)', 'Original(FPA axis)', 'Original(HED axis)');
subplot(3, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Body(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Body(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Body(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Original(roll axis)', 'Original(pitch axis)', 'Original(yaw axis)');
subplot(3, 1, 3);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Mod_Body(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Mod_Body(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Cmd_Mod_Body(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Modified(roll axis)', 'Modified(pitch axis)', 'Modified(yaw axis)');

figure('name', 'Acceleration, Velocity in NED');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_NED(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_NED(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_NED(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Acc(North)', 'Acc(East)', 'Acc(Down)');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Vel_NED(1 : end - 5, 1)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_NED(1 : end - 5, 2)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_NED(1 : end - 5, 3)); grid on;
xlabel('time [sec]'); ylabel('vel [m/s]'); legend('vel(North)', 'vel(East)', 'vel(Down)');

figure('name', 'Acceleration, Velocity in Body');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Body(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Body(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Body(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Acc(roll axis)', 'Acc(pitch axis)', 'Acc(yaw axis)');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Body(1 : end - 5, 1)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Body(1 : end - 5, 2)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Body(1 : end - 5, 3)); grid on;
xlabel('time [sec]'); ylabel('vel [m/s]'); legend('vel(roll axis)', 'vel(pitch axis)', 'vel(yaw axis)');

figure('name', 'Acceleration, Velocity in Velocity Vector');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Wind(1 : end - 5, 1)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Wind(1 : end - 5, 2)/Sim_G); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Acc_Wind(1 : end - 5, 3)/Sim_G); grid on;
xlabel('time [sec]'); ylabel('Acc [G]'); legend('Acc(VV axis 1)', 'Acc(FPA axis)', 'Acc(HED axis)');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Wind(1 : end - 5, 1)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Wind(1 : end - 5, 2)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Vel_Wind(1 : end - 5, 3)); grid on;
xlabel('time [sec]'); ylabel('vel [m/s]'); legend('vel(VV axis 1)', 'vel(FPA axis)', 'vel(HED axis)');

figure('name', 'Torque, Euler Rate');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Torque(1 : end - 5, 1)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Torque(1 : end - 5, 2)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Torque(1 : end - 5, 3)); grid on;
xlabel('time [sec]'); ylabel('lmn [Nxm]'); legend('l', 'm', 'n');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Euler_PQR(1 : end - 5, 1)*Sim_R2D); hold on ;
plot(Result.Time(1 : end - 5), Result.Missile_Euler_PQR(1 : end - 5, 2)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Euler_PQR(1 : end - 5, 3)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('PQR [deg/s]'); legend('P', 'Q', 'R');

figure('name', 'Euler Angles');
subplot(3, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_Euler_cmd(1 : end - 5, 1)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Euler(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('Euler Angle [deg]'); legend('\phi cmd', '\phi');
subplot(3, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Euler_cmd(1 : end - 5, 2)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Euler(1 : end - 5, 2)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('Euler Angle [deg]'); legend('\theta cmd(LOS)', '\theta');
subplot(3, 1, 3);
plot(Result.Time(1 : end - 5), Result.Missile_Euler_cmd(1 : end - 5, 3)*Sim_R2D); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Euler(1 : end - 5, 3)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('Euler Angle [deg]'); legend('\psi cmd(LOS)', '\psi');

figure('name', 'Seeker FOV');
subplot(2, 1, 1);
plot(Result.Time(1 : end - 5), Result.Missile_FOV_Angle(1 : end - 5, 1)*Sim_R2D); grid on;
xlabel('time [sec]'); ylabel('Angle [deg]'); legend('Angle to Target');
subplot(2, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_FOV_InRange(1 : end - 5, 1)); grid on;
xlabel('time [sec]'); ylabel('Logical [-]'); legend('Detection');

figure('name', 'Miss Distance');
subplot(2, 1, 1);
plot(Result.Time(1 : end), Result.Geometry_Speed_Closing(1 : end, 1)); grid on;
xlabel('time [sec]'); ylabel('Velocity [m/s]'); legend('Closing Velocity');
subplot(2, 1, 2);
plot(Result.Time(1 : end), Result.Geometry_Range(1 : end, 1)); grid on;
xlabel('time [sec]'); ylabel('Range [m]'); legend('Miss Distance');

figure('name', 'DACS Thrust');
subplot(3, 1, 1);
plot3([Result.Display_Body(2, 1, 1) Result.Display_Body(2, 2, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 1, 1) Result.Display_Body(1, 2, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 1, 1) Result.Display_Body(3, 2, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 1, 1) Result.Display_Body(2, 3, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 1, 1) Result.Display_Body(1, 3, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 1, 1) Result.Display_Body(3, 3, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 1, 1) Result.Display_Body(2, 5, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 1, 1) Result.Display_Body(1, 5, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 1, 1) Result.Display_Body(3, 5, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 2, 1) Result.Display_Body(2, 4, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 2, 1) Result.Display_Body(1, 4, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 2, 1) Result.Display_Body(3, 4, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 2, 1) Result.Display_Body(2, 6, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 2, 1) Result.Display_Body(1, 6, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 2, 1) Result.Display_Body(3, 6, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 3, 1) Result.Display_Body(2, 4, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 3, 1) Result.Display_Body(1, 4, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 3, 1) Result.Display_Body(3, 4, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 3, 1) Result.Display_Body(2, 7, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 3, 1) Result.Display_Body(1, 7, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 3, 1) Result.Display_Body(3, 7, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 4, 1) Result.Display_Body(2, 8, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 4, 1) Result.Display_Body(1, 8, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 4, 1) Result.Display_Body(3, 8, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 5, 1) Result.Display_Body(2, 6, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 5, 1) Result.Display_Body(1, 6, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 5, 1) Result.Display_Body(3, 6, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 5, 1) Result.Display_Body(2, 7, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 5, 1) Result.Display_Body(1, 7, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 5, 1) Result.Display_Body(3, 7, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 6, 1) Result.Display_Body(2, 8, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 6, 1) Result.Display_Body(1, 8, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 6, 1) Result.Display_Body(3, 8, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;
plot3([Result.Display_Body(2, 7, 1) Result.Display_Body(2, 8, 1)] - Result.Missile_Traj(1, 2), [Result.Display_Body(1, 7, 1) Result.Display_Body(1, 8, 1)] - Result.Missile_Traj(1, 1), -[Result.Display_Body(3, 7, 1) Result.Display_Body(3, 8, 1)] + Result.Missile_Traj(1, 3), 'k'); hold on;

plot3([0 Result.Display_Body_ax1(1, 2)*Disp_Body_Size(1, 1)/2/norm(Result.Display_Body_ax1(1, :))], [0 Result.Display_Body_ax1(1, 1)*Disp_Body_Size(1, 1)/2/norm(Result.Display_Body_ax1(1, :))], -[0 Result.Display_Body_ax1(1, 3)*Disp_Body_Size(1, 1)/2/norm(Result.Display_Body_ax1(1, :))], 'r.-'); hold on;
plot3([0 Result.Display_Body_ax2(1, 2)*Disp_Body_Size(1, 2)/2/norm(Result.Display_Body_ax1(1, :))], [0 Result.Display_Body_ax2(1, 1)*Disp_Body_Size(1, 2)/2/norm(Result.Display_Body_ax1(1, :))], -[0 Result.Display_Body_ax2(1, 3)*Disp_Body_Size(1, 2)/2/norm(Result.Display_Body_ax1(1, :))], 'g.-'); hold on;
plot3([0 Result.Display_Body_ax3(1, 2)*Disp_Body_Size(1, 3)/2/norm(Result.Display_Body_ax1(1, :))], [0 Result.Display_Body_ax3(1, 1)*Disp_Body_Size(1, 3)/2/norm(Result.Display_Body_ax1(1, :))], -[0 Result.Display_Body_ax3(1, 3)*Disp_Body_Size(1, 3)/2/norm(Result.Display_Body_ax1(1, :))], 'b.-'); hold on;

Plt_D1 = plot3(Result.Display_Thrust(2, 1, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 1, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 1, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_D2 = plot3(Result.Display_Thrust(2, 2, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 2, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 2, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_D3 = plot3(Result.Display_Thrust(2, 3, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 3, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 3, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_D4 = plot3(Result.Display_Thrust(2, 4, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 4, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 4, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A1 = plot3(Result.Display_Thrust(2, 5, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 5, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 5, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A2 = plot3(Result.Display_Thrust(2, 6, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 6, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 6, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A3 = plot3(Result.Display_Thrust(2, 7, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 7, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 7, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A4 = plot3(Result.Display_Thrust(2, 8, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 8, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 8, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A5 = plot3(Result.Display_Thrust(2, 9, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 9, 1) - Result.Missile_Traj(1, 1), -Result.Display_Thrust(3, 9, 1) + Result.Missile_Traj(1, 3), 'o'); hold on;
Plt_A6 = plot3(Result.Display_Thrust(2, 10, 1) - Result.Missile_Traj(1, 2), Result.Display_Thrust(1, 10, 1) - Result.Missile_Traj(1, 2), -Result.Display_Thrust(3, 10, 1) + Result.Missile_Traj(1, 3), 'o'); grid on;
axis equal; xlabel('North [m]'); ylabel('East [m]'); zlabel('Up [m]'); legend([Plt_D1 Plt_D2 Plt_D3 Plt_D4 Plt_A1 Plt_A2 Plt_A3 Plt_A4 Plt_A5 Plt_A6], {'1', '2', '3', '4', '5', '6', '7', '8', '9', '10'});

subplot(3, 1, 2);
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 1)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 2)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 3)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 4)); grid on;
xlabel('time [sec]'); ylabel('Divert Thrust [N]'); legend('1', '2', '3', '4');

subplot(3, 1, 3);
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 5)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 6)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 7)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 8)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 9)); hold on;
plot(Result.Time(1 : end - 5), Result.Missile_Thrust(1 : end - 5, 10)); grid on;
xlabel('time [sec]'); ylabel('Attitude Thrust [N]'); legend('5', '6', '7', '8', '9', '10');

figure('name', 'KV Attitude w/Trajectory');
for i = 1 : round(length(Result.Time)/30) : length(Result.Time) - 5
    plot3(Result.Missile_Traj(i, 2), Result.Missile_Traj(i, 1), -Result.Missile_Traj(i, 3), 'b.'); hold on;
    
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_LOS_long(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_LOS_long(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_LOS_long(i, 3)], 'color', [0.5 0.5 0.5]); hold on;
    
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Vel_long(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Vel_long(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Vel_long(i, 3)], 'm'); hold on;
    
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 2, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 2, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 2, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 3, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 3, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 3, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 5, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 5, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 5, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 2, i) Result.Display_Body(2, 4, i)], [Result.Display_Body(1, 2, i) Result.Display_Body(1, 4, i)], -[Result.Display_Body(3, 2, i) Result.Display_Body(3, 4, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 2, i) Result.Display_Body(2, 6, i)], [Result.Display_Body(1, 2, i) Result.Display_Body(1, 6, i)], -[Result.Display_Body(3, 2, i) Result.Display_Body(3, 6, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 3, i) Result.Display_Body(2, 4, i)], [Result.Display_Body(1, 3, i) Result.Display_Body(1, 4, i)], -[Result.Display_Body(3, 3, i) Result.Display_Body(3, 4, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 3, i) Result.Display_Body(2, 7, i)], [Result.Display_Body(1, 3, i) Result.Display_Body(1, 7, i)], -[Result.Display_Body(3, 3, i) Result.Display_Body(3, 7, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 4, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 4, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 4, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 5, i) Result.Display_Body(2, 6, i)], [Result.Display_Body(1, 5, i) Result.Display_Body(1, 6, i)], -[Result.Display_Body(3, 5, i) Result.Display_Body(3, 6, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 5, i) Result.Display_Body(2, 7, i)], [Result.Display_Body(1, 5, i) Result.Display_Body(1, 7, i)], -[Result.Display_Body(3, 5, i) Result.Display_Body(3, 7, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 6, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 6, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 6, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 7, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 7, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 7, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax1(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax1(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax1(i, 3)], 'r'); hold on;
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax2(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax2(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax2(i, 3)], 'g'); hold on;
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax3(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax3(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax3(i, 3)], 'b'); hold on;
    
    plot3([Result.Display_Thrust(2, 1, i) Result.Display_Thrust(2, 11, i)], [Result.Display_Thrust(1, 1, i) Result.Display_Thrust(1, 11, i)], -[Result.Display_Thrust(3, 1, i) Result.Display_Thrust(3, 11, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 2, i) Result.Display_Thrust(2, 12, i)], [Result.Display_Thrust(1, 2, i) Result.Display_Thrust(1, 12, i)], -[Result.Display_Thrust(3, 2, i) Result.Display_Thrust(3, 12, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 3, i) Result.Display_Thrust(2, 13, i)], [Result.Display_Thrust(1, 3, i) Result.Display_Thrust(1, 13, i)], -[Result.Display_Thrust(3, 3, i) Result.Display_Thrust(3, 13, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 4, i) Result.Display_Thrust(2, 14, i)], [Result.Display_Thrust(1, 4, i) Result.Display_Thrust(1, 14, i)], -[Result.Display_Thrust(3, 4, i) Result.Display_Thrust(3, 14, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 5, i) Result.Display_Thrust(2, 15, i)], [Result.Display_Thrust(1, 5, i) Result.Display_Thrust(1, 15, i)], -[Result.Display_Thrust(3, 5, i) Result.Display_Thrust(3, 15, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 6, i) Result.Display_Thrust(2, 16, i)], [Result.Display_Thrust(1, 6, i) Result.Display_Thrust(1, 16, i)], -[Result.Display_Thrust(3, 6, i) Result.Display_Thrust(3, 16, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 7, i) Result.Display_Thrust(2, 17, i)], [Result.Display_Thrust(1, 7, i) Result.Display_Thrust(1, 17, i)], -[Result.Display_Thrust(3, 7, i) Result.Display_Thrust(3, 17, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 8, i) Result.Display_Thrust(2, 18, i)], [Result.Display_Thrust(1, 8, i) Result.Display_Thrust(1, 18, i)], -[Result.Display_Thrust(3, 8, i) Result.Display_Thrust(3, 18, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 9, i) Result.Display_Thrust(2, 19, i)], [Result.Display_Thrust(1, 9, i) Result.Display_Thrust(1, 19, i)], -[Result.Display_Thrust(3, 9, i) Result.Display_Thrust(3, 19, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 10, i) Result.Display_Thrust(2, 20, i)], [Result.Display_Thrust(1, 10, i) Result.Display_Thrust(1, 20, i)], -[Result.Display_Thrust(3, 10, i) Result.Display_Thrust(3, 20, i)], 'color', [0.8500 0.3250 0.0980]); hold on;

%     if Result.Missile_FOV_InRange(i, 1) > 0
%         plot3(Result.Display_FOV_circle_long(2, :, i), Result.Display_FOV_circle_long(1, :, i), -Result.Display_FOV_circle_long(3, :, i), 'g'); hold on;
%     else
%         plot3(Result.Display_FOV_circle_long(2, :, i), Result.Display_FOV_circle_long(1, :, i), -Result.Display_FOV_circle_long(3, :, i), 'r'); hold on;
%     end

    plot3(Result.Target_Traj(i, 2), Result.Target_Traj(i, 1), -Result.Target_Traj(i, 3), 'r.'); grid on;
    
    xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]'); axis equal;
    plot_time = ['T = ' num2str(Result.Time(i, 1)) ' [sec]'];
    Plt_time = text(0.05, 0.95, plot_time, 'Units', 'normalized');
    plot_dist = ['Dist = ' num2str(Geom_range(i, 1)) ' [m]'];
    Plt_dist = text(0.05, 0.9, plot_dist, 'Units', 'normalized'); drawnow;
    delete(Plt_time); delete(Plt_dist);
end

figure('name', 'KV Attitude w/Thrust');
for i = 1 : round(length(Result.Time)/200) : length(Result.Time) - 5
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_LOS_short(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_LOS_short(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_LOS_short(i, 3)], 'color', [0.5 0.5 0.5]); hold on;
    
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Vel_short(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Vel_short(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Vel_short(i, 3)], 'm'); hold on;
    
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 2, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 2, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 2, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 3, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 3, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 3, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 1, i) Result.Display_Body(2, 5, i)], [Result.Display_Body(1, 1, i) Result.Display_Body(1, 5, i)], -[Result.Display_Body(3, 1, i) Result.Display_Body(3, 5, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 2, i) Result.Display_Body(2, 4, i)], [Result.Display_Body(1, 2, i) Result.Display_Body(1, 4, i)], -[Result.Display_Body(3, 2, i) Result.Display_Body(3, 4, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 2, i) Result.Display_Body(2, 6, i)], [Result.Display_Body(1, 2, i) Result.Display_Body(1, 6, i)], -[Result.Display_Body(3, 2, i) Result.Display_Body(3, 6, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 3, i) Result.Display_Body(2, 4, i)], [Result.Display_Body(1, 3, i) Result.Display_Body(1, 4, i)], -[Result.Display_Body(3, 3, i) Result.Display_Body(3, 4, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 3, i) Result.Display_Body(2, 7, i)], [Result.Display_Body(1, 3, i) Result.Display_Body(1, 7, i)], -[Result.Display_Body(3, 3, i) Result.Display_Body(3, 7, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 4, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 4, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 4, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 5, i) Result.Display_Body(2, 6, i)], [Result.Display_Body(1, 5, i) Result.Display_Body(1, 6, i)], -[Result.Display_Body(3, 5, i) Result.Display_Body(3, 6, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 5, i) Result.Display_Body(2, 7, i)], [Result.Display_Body(1, 5, i) Result.Display_Body(1, 7, i)], -[Result.Display_Body(3, 5, i) Result.Display_Body(3, 7, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 6, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 6, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 6, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    plot3([Result.Display_Body(2, 7, i) Result.Display_Body(2, 8, i)], [Result.Display_Body(1, 7, i) Result.Display_Body(1, 8, i)], -[Result.Display_Body(3, 7, i) Result.Display_Body(3, 8, i)], 'k'); hold on;
    
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax1(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax1(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax1(i, 3)], 'r.-'); hold on;
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax2(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax2(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax2(i, 3)], 'g.-'); hold on;
    plot3([Result.Missile_Traj(i, 2) Result.Missile_Traj(i, 2) + Result.Display_Body_ax3(i, 2)], [Result.Missile_Traj(i, 1) Result.Missile_Traj(i, 1) + Result.Display_Body_ax3(i, 1)], -[Result.Missile_Traj(i, 3) Result.Missile_Traj(i, 3) + Result.Display_Body_ax3(i, 3)], 'b.-'); hold on;
    
    plot3(Result.Display_Thrust(2, 1, i), Result.Display_Thrust(1, 1, i), -Result.Display_Thrust(3, 1, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 2, i), Result.Display_Thrust(1, 2, i), -Result.Display_Thrust(3, 2, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 3, i), Result.Display_Thrust(1, 3, i), -Result.Display_Thrust(3, 3, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 4, i), Result.Display_Thrust(1, 4, i), -Result.Display_Thrust(3, 4, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 5, i), Result.Display_Thrust(1, 5, i), -Result.Display_Thrust(3, 5, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 6, i), Result.Display_Thrust(1, 6, i), -Result.Display_Thrust(3, 6, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 7, i), Result.Display_Thrust(1, 7, i), -Result.Display_Thrust(3, 7, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 8, i), Result.Display_Thrust(1, 8, i), -Result.Display_Thrust(3, 8, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 9, i), Result.Display_Thrust(1, 9, i), -Result.Display_Thrust(3, 9, i), 'ko'); hold on;
    plot3(Result.Display_Thrust(2, 10, i), Result.Display_Thrust(1, 10, i), -Result.Display_Thrust(3, 10, i), 'ko'); hold on;
    
    plot3([Result.Display_Thrust(2, 1, i) Result.Display_Thrust(2, 11, i)], [Result.Display_Thrust(1, 1, i) Result.Display_Thrust(1, 11, i)], -[Result.Display_Thrust(3, 1, i) Result.Display_Thrust(3, 11, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 2, i) Result.Display_Thrust(2, 12, i)], [Result.Display_Thrust(1, 2, i) Result.Display_Thrust(1, 12, i)], -[Result.Display_Thrust(3, 2, i) Result.Display_Thrust(3, 12, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 3, i) Result.Display_Thrust(2, 13, i)], [Result.Display_Thrust(1, 3, i) Result.Display_Thrust(1, 13, i)], -[Result.Display_Thrust(3, 3, i) Result.Display_Thrust(3, 13, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 4, i) Result.Display_Thrust(2, 14, i)], [Result.Display_Thrust(1, 4, i) Result.Display_Thrust(1, 14, i)], -[Result.Display_Thrust(3, 4, i) Result.Display_Thrust(3, 14, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 5, i) Result.Display_Thrust(2, 15, i)], [Result.Display_Thrust(1, 5, i) Result.Display_Thrust(1, 15, i)], -[Result.Display_Thrust(3, 5, i) Result.Display_Thrust(3, 15, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 6, i) Result.Display_Thrust(2, 16, i)], [Result.Display_Thrust(1, 6, i) Result.Display_Thrust(1, 16, i)], -[Result.Display_Thrust(3, 6, i) Result.Display_Thrust(3, 16, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 7, i) Result.Display_Thrust(2, 17, i)], [Result.Display_Thrust(1, 7, i) Result.Display_Thrust(1, 17, i)], -[Result.Display_Thrust(3, 7, i) Result.Display_Thrust(3, 17, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 8, i) Result.Display_Thrust(2, 18, i)], [Result.Display_Thrust(1, 8, i) Result.Display_Thrust(1, 18, i)], -[Result.Display_Thrust(3, 8, i) Result.Display_Thrust(3, 18, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 9, i) Result.Display_Thrust(2, 19, i)], [Result.Display_Thrust(1, 9, i) Result.Display_Thrust(1, 19, i)], -[Result.Display_Thrust(3, 9, i) Result.Display_Thrust(3, 19, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    plot3([Result.Display_Thrust(2, 10, i) Result.Display_Thrust(2, 20, i)], [Result.Display_Thrust(1, 10, i) Result.Display_Thrust(1, 20, i)], -[Result.Display_Thrust(3, 10, i) Result.Display_Thrust(3, 20, i)], 'color', [0.8500 0.3250 0.0980]); hold on;
    
    if Result.Missile_FOV_InRange(i, 1) > 0
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 1, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 1, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 1, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 2, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 2, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 2, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 3, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 3, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 3, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 4, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 4, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 4, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 5, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 5, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 5, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 6, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 6, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 6, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 7, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 7, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 7, i)], 'g:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 8, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 8, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 8, i)], 'g:'); hold on;
        plot3(Result.Display_FOV_circle_short(2, :, i), Result.Display_FOV_circle_short(1, :, i), -Result.Display_FOV_circle_short(3, :, i), 'g'); grid on;
    else
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 1, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 1, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 1, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 2, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 2, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 2, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 3, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 3, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 3, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 4, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 4, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 4, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 5, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 5, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 5, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 6, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 6, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 6, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 7, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 7, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 7, i)], 'r:'); hold on;
        plot3([Result.Missile_Traj(i, 2) Result.Display_FOV_short(2, 8, i)], [Result.Missile_Traj(i, 1) Result.Display_FOV_short(1, 8, i)], -[Result.Missile_Traj(i, 3) Result.Display_FOV_short(3, 8, i)], 'r:'); hold on;
        plot3(Result.Display_FOV_circle_short(2, :, i), Result.Display_FOV_circle_short(1, :, i), -Result.Display_FOV_circle_short(3, :, i), 'r'); grid on;
    end
    
    xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]'); axis equal;
    plot_time = ['T = ' num2str(Result.Time(i, 1)) ' [sec]'];
    Plt_time = text(0.05, 0.95, plot_time, 'Units', 'normalized');
    plot_dist = ['Dist = ' num2str(Result.Geometry_Range(i, 1)) ' [m]'];
    Plt_dist = text(0.05, 0.9, plot_dist, 'Units', 'normalized'); drawnow;
    hold off;
end
