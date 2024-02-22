 clear all;
 close all;
 
 s = tf('s');
 
 # Define Controller
 Kp = 0.5;
 Td = 8;
 Kd = Td * Kp;
 
 # Define Blocks
 G = (Kd*s+Kp) / (s^2+Kd*s+Kp);
 C = Kp + Kd*s;
 P = 1/s^2;
 G1 = C*P/(1+C*P);
 
 # Get Step Reponse of Complimentary Sensitivity
 [y, t, x] = step(G1);
 stepinfo(y, t)
 
 # Get Step Reponse of Load Disturbance Sensitivity
 [y, t, x] = step(P/(1+C*P));
 
 # Get Pole zero map
 figure
 pzmap(G1)
 
 figure
 rlocus(C*P)
 
 figure
 step(G);
 hold on
 step(G1, '--');
 hold off
 title ("Step response of a PT2 transfer function");
 
 figure
 bode(G)
 hold on
 bode(G1);
 hold off
 
 [Wn, zeta] = damp(G)