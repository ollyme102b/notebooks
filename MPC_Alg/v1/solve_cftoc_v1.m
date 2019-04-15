function [U,X] = solve_cftoc_v1(A,B,x0,xbar,N)

% state numbers
nx = size(x0,1);
nu = size(B,2);

% assert statements for debugging
assert(size(x0,2)==1, 'x0 must be a col vec');
assert(all(size(x0) == size(xbar)), 'x0 and xbar must have same dimensions');
assert(size(A,2)==size(A,1),'A must be square');
assert(size(A,2)==nx,'Col A must = row x0.');
assert(size(B,1)==nx,'Row B must = row x0.');

% initialize YALMIP variables
X = sdpvar(nx,N+1);
U = sdpvar(nu,N);

% initialize YALMIP constraint
constraints = [];

% initial dynamic constraint
constraints = [constraints,X(:,1) == x0];

% dynamic contraints
for t = 1:N
    constraints = [constraints,X(:,t+1) == A*X(:,t)+B*U(:,t)];
end

% Cost function
XBAR = repmat(xbar,[1,N+1]);
cost = trace((X-XBAR)*(X-XBAR)'+U*U');

% YALMIP options
options = sdpsettings('solver','quadprog','verbose',0);

% YALMIP SOLUTION
solution = optimize(constraints,cost,options);

U = double(U);
X = double(X);
end


