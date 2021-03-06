mdl_fourlink2d;
t = [0:0.1:4.0];
T0 = fkine(fl2d, qn);
T1 = fkine(fl2d, qt);
Traj = ctraj(T0, T1, length(t));
Qik = ikine(fl2d, Traj, qn, [1,1,1,1,0,0], 'verbose=0');
figure(1);
subplot(3,1,1);
plot(t,Qik(:,1)*180/pi,'linewidth',2);
grid on;
title("Joint Angles");
xlabel('Time (s)');
ylabel('Joint 1 (deg)');
subplot(3,1,2);
plot(t,Qik(:,2)*180/pi,'linewidth',2);
grid on;
xlabel('Time (s)');
ylabel('Joint 2 (deg)');
subplot(3,1,3);
plot(t,Qik(:,3)*180/pi,'linewidth',2);
grid on;
xlabel('Time (s)');
ylabel('Joint 3 (deg)');
saveas(gcf(),"octave_ikine2d_plot.jpg","jpg");
fl2d.plotopt = {'base', 'ortho', 'wrist', 'joints'};
figure(2)
plot(fl2d,Qik);
grid on;
saveas(gcf(),"octave_ikine2d_anim.jpg","jpg");
