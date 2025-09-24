
mph_intervals = [0.5 : 0.5 : 10];
measured_rps = [4 5 7 8 9 11 12 13 15 16 17 19 20 21 23 24 26 27 28 30];
miles_to_inches = 63360;
hours_to_seconds = 3600;
diameter = 1.92;
circumference = pi * diameter;

% time in microseconds
upper_speed_lim = [227000 175000 143000 119000 102700 90100 80200 72400 65700 60400 55700 51800 48300 45350 42600 40280 38200 36300 34600 33150];
lower_speed_lim = [225000 172000 141000 118000 102000 89700 79900 72100 65600 60200 55500 51600 48100 45200 42540 40150 38050 36150 34400 33050];

test_data_speed = [0.5 0.6 0.7 0.8 0.9 1.0 1.5 2.0 2.5 3.0 3.5 4.0 4.5 5.0 5.1 5.2 5.3 5.4 5.5 5.6 5.7 5.8 5.9 6.0 6.5 7.0 7.5 8.0 8.5 9.0 9.1 9.2 9.3 9.4 9.5 9.6 9.7 9.8 9.9 10.0];

ideal_PWM = [87.54 105.06 122.52 140.04 157.56 175.08 262.62 350.16 437.7 525.24 612.72 700.26 787.8 875.34 892.86 910.38 927.9 945.36 962.88 980.4 997.92 1015.38 1032.9 1050.42 1137.96 1225.5 1313.04 1400.58 1488.12 1575.66 1593.12 1610.64 1628.16 1645.68 1663.14 1680.66 1698.18 1715.7 1733.22 1750.68];

real_PWM = [95.88 110.91 128.68 145.79 161.77 178.51 263.26 348.55 435.28 524.96 610.08 698.01 784.77 873.74 890.92 908.16 927.57 945.56 962.90 980.30 997.83 1014.58 1033.12 1052.29 1140.13 1229.17 1317.30 1406.59 1493.46 1582.03 1603.27 1619.59 1638.54 1658.25 1674.20 1689.23 1708.60 1727.00 1734.95 1763.10];

avg_one_rev_time = ((upper_speed_lim + lower_speed_lim) / 2) * 10e-6;

%% Calculations for rps and time for a revolution
calculated_rps = (mph_intervals * miles_to_inches) / (hours_to_seconds * circumference);
calculated_one_rev_time = 1 ./ calculated_rps; 

% x = -pi:0.1:pi;
% plot(x, cos(x));

figure(1);
plot(mph_intervals, measured_rps, 'LineWidth', 2)
hold on;
plot(mph_intervals, calculated_rps, "--",'LineWidth', 2)
hold off;
xlim([0 10.5])
ylim([0 33])
grid on
title("Comparison of calculated rps and measured rps for multiple mph values")
xlabel("Miles per hour (mph)")
ylabel("Revolutions per second (rps)")
legend({'measured values', 'calculated values'})


figure(2);
plot(mph_intervals, avg_one_rev_time, mph_intervals, calculated_one_rev_time, 'LineWidth', 2)
grid on
title("Comparison of calculated time for one revolution and measured time for one revolution")
ylabel("Time (microseconds)")
xlabel("Miles per hour (mph)")
legend({'measured values', 'calculated values'})


TREADMILL_SPEEP_PWM = [588 696 804 912 1021  1129  1237  1345  1454  1562  1670  1778  1887  1995  2103  2212  2320  2428  2536  2645];

figure(3);
plot(mph_intervals, TREADMILL_SPEEP_PWM,'LineWidth', 2)
hold on;
plot(test_data_speed, ideal_PWM, 'LineWidth', 2)
plot(test_data_speed, real_PWM, "--",'LineWidth', 2)
hold off;
grid on
title("PWM Values vs MPH")
ylabel("PWM values")
xlabel("Miles per hour (mph)")
legend({'old values','ideal values', 'new values'})
