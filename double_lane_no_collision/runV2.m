% MATLAB initialization
clc
close all

ncars = 5;
nlane = 1;

% Simulation integrator time step
tol = 2;
dt_sim = 5*10^-tol;

%% Car initialization
% Create car queue
path        = [0, 50];
init_lane   = 0;
leader      = 0;
inter       = [0, 0];

% Specified conditions
%
mass        = {1,1,1,1,1};
collSOI     = {3,3,3,3,3};
commSOI     = {1,1,1,1,1};
state       = {[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]};
vLim        = {[10,10],[60,60],[5,5],[40,40],[80,80]};
fLim        = {[10,0],[10,0],[10,0],[10,0],[10,0]};
force       = {[10,0],[20,0],[10,0],[30,0],[30,0]};

for i = 1:ncars
    c{i} = carV2( i, mass{i}, collSOI{i}, commSOI{i}, vLim{i}, ...
        force{i}, fLim{i}, state{i}, init_lane, path, leader, inter );
end
%}

% Random conditions
%{
for i = 1:ncars
    mass    = 1;
    collSOI = 1;
    commSOI = 1;
    state   = [0,0,0,0];
    vLim    = [0, 5*randi(10)];
    fLim    = [50,0];
    force   = [10*randi(fLim(1)), 0];
    c{i} = carV2( i, mass, collSOI, commSOI, vLim, ...
        force, fLim, state, init_lane, path, leader, inter );
end
%}


%% Simulation
figure
hold on
grid on
axis equal
axis([path(1)-1 path(2)+1 -1 7])

% Demark the lanes
x = linspace(0,50);
for i = 1:nlane+1
    plot(x,(i-0.5)*ones(size(x)),'r-')
end

% Initialize all car objects
for i = 1:ncars
    car{i} = plot(c{i}.state(1),c{i}.state(2),'ko','MarkerSize',5);
end

% Initialize lane_queue
lane_queue = cell(nlane,1);
cur_time = 0;
queue_pos = 0;
min_pos = 0;
iter = 1;
while min_pos <= path(2)
    
    % TO-DO: Calculate the distance between every agent (square symmetric
    % matrix)
    
    if (queue_pos <= ncars) %&& (c{queue_pos}.state(1) > c{queue_pos}.collSOI+c{queue_pos+1}.collSOI)
        
        % Increment queue position
        queue_pos = queue_pos + 1;
        if queue_pos > ncars
            queue_pos = ncars;
        end
        
        % Latest car and time of car in lane
        latest = zeros(nlane,2); % [ID, time]
        for i = 1:nlane
            if ~isempty(lane_queue{i})
                latest(i,1) = lane_queue{i}.ID;
                latest(i,2) = (lane_queue{i}.path(2) - ...
                    lane_queue{i}.state(1)) / ...
                    lane_queue{i}.vLim(1);
                disp(lane_queue{i}.state)
            end
        end
        
        %disp(latest)
        
        % Current car time
        tc = (c{queue_pos}.path(2) - c{queue_pos}.state(1))/c{queue_pos}.vLim(1);
        
        % Get the proper lane from min time lane
        time = latest(:,2) - tc;
        [~, t_idx] = min(time);
        
        % Set the lane of the current car to the next car
        c{queue_pos}.lane = t_idx(1);
        
        % Set the y-position to be the lane
        c{queue_pos}.state(2) = c{queue_pos}.lane;
        
        % Add to the lane_queue
        for i = 1:2
            lane_queue{t_idx(1)} = c{queue_pos};
        end
        
    end
    
    if queue_pos > ncars
        queue_pos = ncars;
    end
    
    for i = 1:queue_pos
        % Mechanics with constant force
        c{i} = c{i}.statePropagate(dt_sim);
        
        % Check if velocity limits are crossed
        c{i} = c{i}.velCheck;
        
        % Check if the car is in the intersection
        c{i} = c{i}.interSet;
        
        % Check for collision
        for j = queue_pos:-1:2
            for k = j-1:-1:1
                c{j} = c{j}.checkColl(c{k});
            end
        end
        
        % Plotting
        set(car{i}, 'Xdata', c{i}.state(1))
        set(car{i}, 'Ydata', c{i}.state(2))
        drawnow
        
        M(iter) = getframe;
        
    end
    
    % Find the minimium x-position of the set of cars
    car_pos = [];
    for i = 1:ncars
        car_pos = [car_pos, c{i}.state(1)];
    end
    min_pos = min(car_pos);
    
    % Increment timestep
    cur_time = cur_time + dt_sim;
    
    iter = iter + 1;
    
end

% Make movie
movie(M,1)
myVideo = VideoWriter('multi.avi');
uncompressedVideo = VideoWriter('multi.avi', 'Uncompressed AVI');
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);