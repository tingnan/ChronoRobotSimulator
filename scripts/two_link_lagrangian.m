clear;
syms t0 p1 l m
syms t0dt p1dt

t0 = 0;
p1 = 0;
l = 1;
m = 1;

J = [-l/2*sin(t0), 0;
      l/2*cos(t0), 0;
     -l*sin(t0)-l/2*sin(t0+p1),-l/2*sin(t0+p1);
      l*cos(t0)+l/2*cos(t0+p1), l/2*cos(t0+p1);
      1, 0
      1, 1];
  
Jdt = [-l/2*cos(t0)*t0dt, 0;
        -l/2*sin(t0)*t0dt, 0;
        -l*cos(t0)*t0dt-l/2*cos(t0+p1)*(t0dt+p1dt),-l/2*cos(t0+p1)*(t0dt+p1dt);
        -l*sin(t0)*t0dt-l/2*sin(t0+p1)*(t0dt+p1dt),-l/2*sin(t0+p1)*(t0dt+p1dt);
        0, 0
        0, 0];
    
Mc = diag([m,m,m,m,m*l^2/12,m*l^2/12]);

Mq = transpose(J)*Mc*J;
Cqqdt = transpose(J)*Mc*Jdt*[t0dt;p1dt];


Jt = transpose(J);
Jv = Jt(:,1:4);
Jw = Jt(:,5:6);

%%

f = [0;0;0;1];
tau = -pinv(Jw)*Jv*f

Q = Jw*tau+Jv*f;

