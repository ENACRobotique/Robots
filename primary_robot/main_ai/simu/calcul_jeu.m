L = 310;

% droit
h0 = 69;
h1 = 60;
h2 = 53;
h3 = 35.5;

theta_1 = asin((h2 - h0)/L);
theta_2 = asin((h3 - h1)/L);

theta_d = abs(theta_1 - theta_2)*180/pi

% gauche
h0 = 124;
h1 = 43.5;
h2 = 172;
h3 = 130;

theta_1 = asin((h2 - h0)/L);
theta_2 = asin((h3 - h1)/L);

theta_g = abs(theta_1 - theta_2)*180/pi
