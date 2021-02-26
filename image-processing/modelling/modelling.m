% percent_120: percent values for 120 cm length
pixel_to_cm = 120./(640.*percent_120/100);
% percent_211: percent values for 211 cm length
pixel_to_cm_2 = 211./(640.*percent_211/100);

% distance_120: distance values for 120 cm length
plot(distance_120, pixel_to_cm)
axis square
hold on

% distance_211: distance values for 211 cm length
plot(distance_211, pixel_to_cm_2)

% Linear regression
p = polyfit(distance, pixel_to_cm, 1)
xi = min(distance):0.01:800;
yi = polyval(p, xi);
hold on