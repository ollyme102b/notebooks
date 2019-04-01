function [xf,yf] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1,varargin)
% calculates the (xf,yf) position given (xm,ym), l, and the constraint line
% from (x0,y0) to (x1,y1) that is closest to either (1) the "previous"
% point (xp,yp) given by varargin or (2) the origin (0,0).

if nargin == 9 % previous position provided
    xp = varargin{1};
    yp = varargin{2};
elseif nargin == 7 % assume near origin (0,0)
    xp = 0;
    yp = 0;
else
    error('circle_line_ineff my only take 7 or 9 arguments');
end

% center problem around (xm, ym)
x0 = x0 - xm;
y0 = y0 - ym;
x1 = x1 - xm;
y1 = y1 - ym;
xp = xp - xm;
yp = yp - ym;

if x0-x1 == 0 % slope undefined case
    xfs = [x0,x0]; % solution must have same x value
    yfs = [ym + sqrt(l^2-x0^2),ym - sqrt(l^2-x0^2)]; % solve circle intersect verical line y values
else % slope defined case
    m = (y1-y0)/(x1-x0); % slope
    b = y0 - m*x0; % y intercept
    
    xfs = roots([1+m^2,2*m*b,b^2-l^2]); % solve circle intersect line x values
    yfs = m*xfs + b; % deduce y values from x values
end

if ~isreal(xfs) || ~isreal(yfs)
    xf = [];
    yf = [];
    return;
end

if   pdist([xfs(1),yfs(1); xp, yp],'euclidean')...
   < pdist([xfs(2),yfs(2); xp, yp],'euclidean') % determine better point of the 2 solutions
    xf = xfs(1);
    yf = yfs(1);
else
    xf = xfs(2);
    yf = yfs(2);
end

% uncenter the problem from (xm,ym)
xf = xf + xm;
yf = yf + ym;
end