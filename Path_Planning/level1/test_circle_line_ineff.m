% Unit test for circle_line_ineff

tol = 1e-5;

%% test varargin
assert(circle_line_ineff(1, 1, 2, -10, 1, 10, 1) == ...
       circle_line_ineff(1, 1, 2, -10, 1, 10, 1, 0, 0));
assert(circle_line_ineff(1, 1, 2, -10, 1, 10, 1) ~= ...
       circle_line_ineff(1, 1, 2, -10, 1, 10, 1, 3, 3));
   
%% test alg on basic setup
[xt,yt] = circle_line_ineff(0, 0, 2, -10, 1, 10, 1, 10, 1);
assert(abs(xt - sqrt(3))<tol);
assert(abs(yt - 1)<tol);

[xt,yt] = circle_line_ineff(0, 0, 2, -10, 1, 10, 1, -10, 1);
assert(abs(xt + sqrt(3))<tol);
assert(abs(yt - 1)<tol);

[xt,yt] = circle_line_ineff(0, 0, 2, 1, 10, 1, -10, 1, 10);
assert(abs(xt - 1)<tol);
assert(abs(yt - sqrt(3))<tol);

[xt,yt] = circle_line_ineff(0, 0, 2, 1, 10, 1, -10, 1, -10);
assert(abs(xt - 1)<tol);
assert(abs(yt + sqrt(3))<tol);

%% test (xm,ym) transformation on random data
rng default; % set rand seed
N = 100; % randomized test count
min = -100; % random range max
max = 100; % random range min
for i = 1:N
    xm = randi([min,max]);
    ym = randi([min,max]);
    l  = randi([0,max]);
    x0 = randi([min,max]);
    y0 = randi([min,max]);
    x1 = randi([min,max]);
    y1 = randi([min,max]);
    xp = randi([min,max]);
    yp = randi([min,max]);
    
    [xa,ya] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1, xp, yp);
    [xb,yb] = circle_line_ineff( 0,  0, l, x0-xm, y0-ym, x1-xm, y1-ym, xp-xm, yp-ym);
    
    if ~isempty(xa)
        assert(xa == xb+xm);
        assert(ya == yb+ym);
    end
end
   
   