
pymodule = py.importlib.import_module('circleLineIneff');

N = 1000; % number of random tests

for i = 1:N
    
    xm = randi([-10 10]);
    ym = randi([-10 10]);
    l = randi([1 10]);
    x0 = randi([-10 10]);
    y0 = randi([-10 10]);
    x1 = randi([-10 10]);
    y1 = randi([-10 10]);
    xp = randi([-10 10]);
    yp = randi([-10 10]);
    
   [xmat,ymat] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1, xp, yp);
   
   p_tuple = pymodule.circle_line_ineff(xm, ym, l, x0, y0, x1, y1, xp, yp);
   xpy = p_tuple{1};
   ypy = p_tuple{2};
   
   assert((strcmpi(class(xpy),'py.NoneType') && isempty(xmat))...
       || (xmat == xpy && ymat == ypy), 'Python does not match Matlab');
end



