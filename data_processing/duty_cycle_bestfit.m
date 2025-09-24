close all;
clear;

speed = 1:10;
duty = [18.762, 26.260, 33.261, 40.786, 48.259, 55.294, 62.795, 69.821, 77.310, 84.758];

coefficients = polyfit(speed, duty, 1);

speedFit = linspace(min(speed), max(speed), 20);
dutyFit = polyval(coefficients, speedFit);

figure;
plot(speed, duty, 'bO', LineWidth=1.5);
hold on;
plot(speedFit, dutyFit, 'r', LineWidth=1.5);
xlabel('Speed (MPH)');
ylabel('Duty Cycle (%)');
title('Drive Motor Linear Regression');
legend('Experimental', 'Regression', Location='northwest');