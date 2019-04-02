%% Simulatation with Viszualized

% Initialize Folly position
xf = 0;
yf = 0;

% Initialize Molly position
xm = -4;
ym = -8;

% object length
l = 2;

% constraint line 
x0 = -10;
y0 = -4;
x1 = 4;
y1 = -10;

% simulation time
T = 600;

h = figure;
set(h,'KeyPressFcn',@moveMolly);
global s;
s = [0,0];
for t = 1:T
    xm = xm + s(1);
    ym = ym + s(2);
    s = [0,0];
    
    [xf,yf] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1, xf, yf);
   
    hold on;
    axis equal;
    plot([x0,x1],[y0,y1],'k-','linewidth',1);
    plot([xm,xf],[ym,yf],'b-','linewidth',2);
    plot(xm,ym,'o','color',[0.4660, 0.6740, 0.1880]);
    plot(xf,yf,'ro');
    legend('Path','Load','Molly','Folly');
    title('Level 1 Path Planner');
    pause(0.1);
    clf
end

function moveMolly(~,evnt)
    global s;
    M = 0.05;
    switch evnt.Key
        case 'rightarrow'
            s = [M,0];
        case 'leftarrow' 
            s = [-M,0];
        case 'uparrow' 
            s = [0,M];
        case 'downarrow'
            s = [0,-M];
        otherwise
    end
end

