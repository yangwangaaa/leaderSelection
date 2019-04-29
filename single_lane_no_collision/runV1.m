
% MATLAB initialization
clc
clear
close all

% Simulation integrator time step
tol = 2;
dt_sim = 5*10^-tol;

%% Car initialization
% Create car queue
ncars = 5;
path = [0, 50];
leader = 1;
inter = [0, 0];
state = [0, 0, 0, 0];

mass = {1,1,1,1,1};
collSOI = {3,3,3,3,3};
commSOI = {1,1,1,1,1};
vLim = {[0,10],[0,20],[0,50],[0,40],[0,80]};
fLim = {[10,0],[10,0],[10,0],[10,0],[10,0]};
force = {[10,0],[20,0],[10,0],[30,0],[30,0]};

% Initialize all car data
for i = 1:ncars
    %car(ID, mass, collSOI, commSOI, vLim, force, fLim, state, path, leader, inter)
    c{i} = carV1( i, mass{i}, collSOI{i}, commSOI{i}, vLim{i}, force{i}, fLim{i}, state, path, leader, inter );
    
end


%% Simulation
figure
hold on
grid on
axis equal
axis([path(1)-1 path(2)+1 -1 1])

%{
% Demark the lanes
x = linspace(0,50);
for i = 1:nlane+1
    plot(x,(i-0.5)*ones(size(x)),'r-')
end
%}

% Initialize all car objects
for i = 1:ncars
    car{i} = plot(c{i}.state(1),c{i}.state(2),'ko','MarkerSize',15);
end

cur_time = 0;
queue_pos = 1;
car_queue = 1;
iter = 1;
min_pos = 0;

while min_pos <= path(2)
    
    if (queue_pos < ncars) && ...
            (c{queue_pos}.state(1) > c{queue_pos}.collSOI+c{queue_pos+1}.collSOI)
        queue_pos = queue_pos + 1;
        car_queue = [car_queue, queue_pos];
    end
    
    for i = car_queue
        % Mechanics with constant force
        c{i} = c{i}.statePropagate(dt_sim);
        
        % Check if velocity limits are crossed
        c{i} = c{i}.velCheck;
        
        % Check if the car is in the intersection
        c{i} = c{i}.interSet;
        
        % Check for collision
        for j = queue_pos:-1:2
            c{j} = c{j}.checkColl(c{j-1});
        end
        
        % Plotting
        set(car{i}, 'Xdata', c{i}.state(1))
        set(car{i}, 'Ydata', c{i}.state(2))
        drawnow
        
        M(iter) = getframe;
        
        % Check if car has hit the end
        %{
        if c{i}.state(1) > c{i}.path(2)
            delete(car{i})
            car_queue = car_queue(1:end-1);
        end
        %}
        
    end
    
    % Find the minimium x-position of the set of cars
    car_pos = [];
    for i = 1:ncars
        car_pos = [car_pos, c{i}.state(1)];
    end
    min_pos = min(car_pos);
    
    % Increment timestep
    cur_time = round(cur_time + dt_sim,tol);
    
    iter = iter + 1;
    
end

movie(M,1)
myVideo = VideoWriter('single.avi');
uncompressedVideo = VideoWriter('single.avi', 'Uncompressed AVI');
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);
