clear all;

os = 5;
damping = -log(os/100)/sqrt( pi^2 + log(os/100)^2 )

##Ts = 0.74;
##nat_freq = -log(0.02*sqrt(1-damping^2)) / (damping * Ts);
##% nat_freq = -log(0.02*sqrt(1-damping^2)) / (damping * Ts); % No Approximaition

Tp = 0.3;
nat_freq = pi / (Tp*sqrt(1-damping^2))

pole_loc = roots([1, 2*damping*nat_freq, nat_freq^2])

pole_choice = 12.1;
##syms s
##(s - pole_choice)*(s - pole_loc(1))*(s - pole_loc(2))

