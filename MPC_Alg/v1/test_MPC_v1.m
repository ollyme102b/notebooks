%% Simulatation with Viszualized
% clean up
clear all;
close all;

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

% Initialize Folly position
[xf,yf] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1);

% simulation 
T = 600; % Total time
dt = 1; % time per iteration
sigma = 0; % simulation noise standard deviation

% plot vector function
plotvector = @(c,v,color) quiver(c(1),c(2),v(1),v(2),'color',color,'linewidth',2);

h = figure;
set(h,'KeyPressFcn',@moveMolly);
global s;
s = [0,0];
for t = 1:T
    xm = xm + s(1);
    ym = ym + s(2);
    s = [0,0];
    
    % find optimal folly position
    [xfb,yfb] = circle_line_ineff(xm, ym, l, x0, y0, x1, y1, xf, yf);
    
    % find optimal command
    [U,~] = solve_cftoc_v1(eye(2),dt*eye(2),[xf;yf],[xfb;yfb],10);
    vc = U(:,1); % optimal velocity command
    
    % plot state before actuation 
    hold on;
    axis equal;
    plot([x0,x1],[y0,y1],'k--','linewidth',1); % constraint path
    plot([xm,xfb],[ym,yfb],'b-','linewidth',2); % object lifted
    plot(xm,ym,'o','color',[0.4660, 0.6740, 0.1880]); % Molly Position
    plot(xfb,yfb,'o','color',[0.3,0,0.5]); % Folly optimal position
    plot(xf,yf,'r.'); % Folly Actual Position
    plotvector([xf;yf],10*vc, [1,0.5,0]); % Folly velocity command
    legend('Path','Load','Molly','Folly Optimal','Folly Position','Folly Velocity Command');
    title('Level 1 Path Planner Level 1 MPC');
    pause(1);
    clf
    
    % simulate actuation of optimal command
    xf = xf + dt*vc(1) + normrnd(0,sigma);
    yf = yf + dt*vc(2) + normrnd(0,sigma);
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

