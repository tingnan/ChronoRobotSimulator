clear
n = 4;
l = 1;
m = 1;

M = blkdiag(diag(m*ones(2*n,1)),diag(m*l^2/12*ones(n, 1)));

% q = sym('q', [n, 1]);
% dqdt = sym('dqdt', [n, 1]);
q = zeros(n+2, 1);

J(1,:) = [-l/2*sin(q(1)), zeros(1, n-1)];
J(2,:) = [ l/2*cos(q(1)), zeros(1, n-1)];

for i = 2:n
    tmp1 = cumsum(transpose(q(1:i-1)));
    tmp2 = sum(q(1:i));
    J(2*i-1,:) = [-l*sin(tmp1),-l/2*sin(tmp2), zeros(1, n-i)];
    J(2*i-0,:) = [ l*cos(tmp1), l/2*cos(tmp2), zeros(1, n-i)];
end
J = cumsum(J, 2, 'Reverse');
for i = 2*n+1:3*n
    J(i,1:i-2*n) = 1;
end
J = [J, zeros(3*n, 2)];
J(1:2*n, n+1:n+2) = repmat([1,0;0,1], [n, 1]);


Jt = transpose(J);
Jvt = Jt(:,1:2*n);
Jwt = Jt(:,2*n+1:3*n);

Mq = Jt*M*J;

% in the CoM frame, we first compute the transform matrix
Jth = diag(ones(n, 1));
Jth(1,:) = -(n:-1:1)/n;
Jth(1,1) = 1;
Jdx = -sum(J(1:2:2*n-1, 1:n))/n * Jth;
Jdy = -sum(J(2:2:2*n-0, 1:n))/n * Jth;
JJ = [[Jth, zeros(n, 2)]; 
      [Jdx, 1, 0]; 
      [Jdy, 0, 1]];

Jc = J * JJ;

Jct = transpose(Jc);
Jcvt = Jct(:,1:2*n);
Jcwt = Jct(:,2*n+1:3*n);

Mqc = Jct*M*Jc;
%
%%
clear
n = 4;
l = 1;
m = 1;

M = blkdiag(diag(m*ones(2*n,1)),diag(m*l^2/12*ones(n, 1)));

% q = sym('q', [n, 1]);
% dqdt = sym('dqdt', [n, 1]);
q = pi/4*zeros(n+2, 1);

for i = 1:n
    J(2*i-1,i) = -l/2*sin(q(i));
    J(2*i-0,i) =  l/2*cos(q(i));
end

for i = 2:n
    J(2*i-1,1:i-1) = -l*sin(q(1:i-1));
    J(2*i-0,1:i-1) =  l*cos(q(1:i-1));
end

for i = 2*n+1:3*n
    J(i,i-2*n) = 1;
end
J = [J, zeros(3*n, 2)];
J(1:2:2*n-1, n+1) = 1;
J(2:2:2*n-0, n+2) = 1;

% in the CoM frame, we first compute the transform matrix
Jth(1,:) = [ones(1,n)/n, 0, 0];
for i = 2:n
    Jth(i,i-1) = -1;
    Jth(i,i) = 1;
end
Jdx = [sum(J(1:2:2*n-1, 1:n))/n, 1, 0];
Jdy = [sum(J(2:2:2*n-0, 1:n))/n, 0, 1];
JJ = inv([Jth;Jdx;Jdy]);
Jc = J * JJ;

Jct = transpose(Jc);
Jcvt = Jct(:,1:2*n);
Jcwt = Jct(:,2*n+1:3*n);

Mqc = Jct*M*Jc;




%%
f = [0;1/3;0;-1;0;1;0;-1/3];
Qvc = Jcvt*f;
linsolve(Mqc, Qvc);
Qv = Jvt*f;
%%

MM = Mq;
QQ = Qv;

M3 = [MM(1,1),MM(1,n+1:n+2); 
      MM(n+1:n+2,1),MM(n+1:n+2, n+1:n+2)];
Q3 = [QQ(1);QQ(n+1:n+2)];
q3 = linsolve(M3, Q3);


Qint = [MM(1:end,1),MM(1:end,n+1:n+2)]*q3 - QQ;
linsolve(MM, QQ + Qint)
