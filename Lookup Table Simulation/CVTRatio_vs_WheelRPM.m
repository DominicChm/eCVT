clc;
clear all;
close all;

% Engine Speed
e = 3600;                               % RPM
% Gearbox Ratio
gb = 6.839;

% Wheel RPM
w = 0:1:800;

% CVT Ratio
cvt = e ./ (gb * w);
for i = 1:length(cvt)
    if cvt(i) > 4
        cvt(i) = 4;
    end
    if cvt(i) < 0.75
        cvt(i) = 0.75;
    end
end

% % Wheel RPM to Vehicle Speed
% rpm = w;
% speed = rpm * (2*pi)/60 * 11/12;        % ft/s
% speed = speed * 15/22;                  % mph

% Plot
plot(w, cvt, 'k', 'LineWidth', 2);
title('CVT Ratio vs. Rear Wheel Speed');
xlabel('Rear Wheel Speed (RPM)');
ylabel('CVT Ratio');