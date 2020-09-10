pixel_to_cm = 120./(640.*yuzde120/100);

pixel_to_cm2 = 211./(640.*yuzde211/100);

plot(mesafe, pixel_to_cm)
axis square

hold on
plot(mesafe2, pixel_to_cm2)

p = polyfit(mesafe, pixel_to_cm, 1)
xi = min(mesafe):0.01:800;
yi = polyval(p, xi);

hold on

plot(xi, yi)