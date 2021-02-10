MSA_main
VJM_main
clearvars -except all_deflections_with_forces_MSA all_deflections_with_forces_VJM x_all y_all z_all VJM_time MSA_time
clc

titles = ['x', 'y', 'z'];
for i = 1:3
all_deflections_MSA = cell2mat(all_deflections_with_forces_MSA(i));
all_deflections_VJM = cell2mat(all_deflections_with_forces_VJM(i));
difference = all_deflections_MSA - all_deflections_VJM;
defference_mag = (difference(1,:).^2 + difference(2,:).^2 + difference(3,:).^2).^0.5;

figure;
titles = ['x', 'y', 'z'];
scatter3(x_all, y_all, z_all,40,defference_mag,'filled')    % draw the scatter plot
ax = gca;
ax.XDir = 'reverse';
%     view(-31,14)
xlabel("X")
ylabel("Y")
zlabel("Z")
title(sprintf(" Difference between MSA and VJM, with force in %s directions",titles(i)))

colormap(hot);
cb = colorbar;                                     % create and label the colorbar
cb.Label.String = 'Deflections Magnitude';
end