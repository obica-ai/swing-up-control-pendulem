% keep K < 300

%calculate a
wnp_meas = 8.362919;
a = wnp_meas*wnp_meas;

% A matrix
A = [ 0 1 0 0 ;
      a 0 0 0 ;
      0 0 0 1 ;
      0 0 0 0];

% down controller
A_d = [ 0 1 0 0 ;
      -a 0 0 0 ;
      0 0 0 1 ;
      0 0 0 0];

% B matrix
B = [ 0; -0.87; 0; 163];
C = [1 0 0 0 ; 0 0 1 0];
At = transpose(A);
Ct = transpose(C);

% poles
% wnp = 9, zeta = 0.5 --> use 2nd order sys denominator to solve for poles 
p = [-3.9+8.65i, -3.9-8.65i, -3.9, 0] ;
p_observed = [-50+8.65i, -50-8.65i, -50, -45] ;

p_unstable = [5+8.65i, 5-8.65i, 5, 0] ;
% compute K
rank(ctrb(A,B));
K = place (A,B, p);

K_d = place (A_d,B, p);
K_unstable = place (A_d,B, p_unstable);
L1 = place (At,Ct, p_observed);
%L = transpose(L1)

A_dt = transpose(A_d);

L_d = place(A_dt,Ct,p_observed);
L_dt = transpose(L_d);

% check stability
X_new = (A_d-L_dt*C);
eig(X_new)