mdl_puma560;
[t, q, qd] = fdyn(nofriction(p560), 1.0);
figure(1);
subplot(3,1,1);
plot(t,q(:,1)*180/pi,'linewidth',2);
grid on;
title("Joint Angles");
xlabel('Time (s)');
ylabel('Joint 1 (deg)');
subplot(3,1,2);
plot(t,q(:,2)*180/pi,'linewidth',2);
grid on;
xlabel('Time (s)');
ylabel('Joint 2 (deg)');
subplot(3,1,3);
plot(t,q(:,3)*180/pi,'linewidth',2);
grid on;
xlabel('Time (s)');
ylabel('Joint 3 (deg)');
saveas(gcf(),"octave_fdyn3d_plot.jpg","jpg");
p560.plotopt = {'base', 'ortho', 'nowrist', 'joints'};
plot(p560,q);
saveas(gcf(),"octave_fdyn3d_anim.jpg","jpg");
