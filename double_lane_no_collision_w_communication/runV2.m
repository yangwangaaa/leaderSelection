% Simulation
figure
hold on
grid on
axis([path(1)-1 path(3)+1 0 nlane+1])

% Demarcate the lanes
x = linspace(0,path(3));
for i = 1:nlane+1
    plot(x,(i-0.5)*ones(size(x)),'r-','LineWidth',3)
end
plot([path(2), path(2)], [0, nlane+1], 'k--')

% Initialize all car objects
car     = cell(ncars,1);
clabel  = cell(ncars,1);
for i = 1:ncars
    car{i}  = plot(c{i}.state(1),c{i}.state(2),'ko','MarkerSize',20,...
        'MarkerFaceColor',[0 0 0]);
    clabel{i} = text(c{i}.state(1),c{i}.state(2),num2str(i));
end

% Initialize all communication visualization
for i = 1:ncars
    for j = 1:ncars
        lin(i,j) = plot(0,0,'g-','LineWidth',1);
        hidem(lin(i,j))
    end
end

% Initialize lane_queue
lane_queue      = cell(nlane,1);
cur_time        = 0;
queue_pos       = 1;
min_pos         = 0;
iter            = 1;

% Initialize a queue for all finished cars
finished        = cell(ncars,1);
finish_count    = 1;

% Find the minimium x-position of the set of cars
car_pos = zeros(ncars,1);

% Car 1 initialization
lane_queue{1}   = c{1};
c{1}.state(2)   = 1;

% Initialize latest car in lanes
latest          = zeros(nlane,3);   % [ID, time, velocity]
latest(1,1)     = c{1}.ID;          % First car
latest(1,2)     = (c{1}.path(2) - c{1}.state(1)) / c{1}.vLim;
latest(1,3)     = c{1}.vLim;

while finish_count <= ncars
    
    % Display results
    clc
    fprintf('Current queue position: %1.0f\n\n',queue_pos)
    
    % Latest car and time of car in each lane
    fprintf('Lane\tCar\tTime\tVelocity\n')
    for i = 1:nlane
        if isempty(lane_queue{i}) || ~isvalid(lane_queue{i})
            latest(i,1) = 0;
            latest(i,2) = -inf;
            latest(i,3) = inf;
        elseif ~isempty(lane_queue{i})
            latest(i,1) = lane_queue{i}.ID;
            latest(i,2) = (lane_queue{i}.path(2) - ...
                lane_queue{i}.state(1)) / lane_queue{i}.vLim;
            latest(i,3) = lane_queue{i}.state(3);
        end
        fprintf('%1.0f\t%1.0f\t%f\t%f\n',i,latest(i,1),latest(i,2),latest(i,3))
    end
    
    % Generate a random number to tell the next car to go
    go_car      = rand >= 0.9;
    
    % Lane selection
    if queue_pos < ncars && go_car
        
        if c{queue_pos}.state(1) > c{queue_pos}.collSOI + c{queue_pos+1}.collSOI
            
            % Add car to queue
            queue_pos   = queue_pos + 1;
            
            % Current car time and velocity
            tc          = (c{queue_pos}.path(2) - c{queue_pos}.state(1)) / c{queue_pos}.vLim;
            vc          = c{queue_pos}.vLim;
            
            % Get the proper lane from min time lane
            time_delta      = abs(latest(:,2) - tc);
            vel_delta       = vc - latest(:,3);
            absvel_delta    = 0.5 * (abs(vel_delta) + (vel_delta));
            if strcmp(lane_selection,'v1')
                tmin        = min(time_delta);
                t_idx       = find(tmin==time_delta);
            elseif strcmp(lane_selection,'v2')
                vmin        = min(absvel_delta);
                v_idx       = find(vmin==absvel_delta);
                tmin        = min(abs(vel_delta(v_idx)));
                t_idx       = v_idx(tmin==abs(vel_delta(v_idx)));
            elseif strcmp(lane_selection,'greedy')
                tmin        = min(latest(:,2));
                t_idx       = find(tmin==latest(:,2));
            end
            
            % Set the y-position to be the lane
            c{queue_pos}.state(2) = t_idx(1);
            
            % Add to the lane_queue
            for i = 1:2
                lane_queue{t_idx(1)} = c{queue_pos};
            end
            
        end
        
    end
    
    % Iterate for every car in the current queue
    for i = 1:queue_pos
        
        if c{i}.state(1) < c{i}.path(2)
            
            % Propagate the state
            c{i}.state = c{i}.state + [c{i}.state(3:4), c{i}.accel, 0]*dt_sim;
            
            % Check if velocity limits are crossed
            if c{i}.state(3) >= c{i}.vLim
                c{i}.accel  = 0;
            else
                c{i}.accel  = c{i}.aLim;
            end
            
            % Collision and communication
            for j = queue_pos:-1:2
                
                for k = j-1:-1:1
                    
                    if ((c{j}.state(1) < c{j}.path(2)) && (c{k}.state(1) < c{k}.path(2)))
                        
                        % Check for collision (only in same lane)
                        if (abs(c{k}.state(1)-c{j}.state(1)) <= c{j}.collSOI+c{k}.collSOI) && (c{j}.state(2) == c{k}.state(2))
                            c{j}.state(3) = c{k}.state(3);
                            c{j}.accel = 0;
                        end
                        
                        % Check for communication (only in same lane)
                        if abs(c{j}.state(1)-c{k}.state(1)) <= max([c{j}.commSOI,c{k}.commSOI])
                            for l = 1:2
                                c{j}.know{k,1} = c{k};
                                c{j}.know{k,2} = cur_time;
                                c{k}.know{j,1} = c{j};
                                c{k}.know{j,2} = cur_time;
                            end
                            X = [c{j}.state(1), c{k}.state(1)];
                            Y = [c{j}.state(2), c{k}.state(2)];
                            
                            % Visualize communication link
                            showm(lin(j,k))
                            set(lin(j,k), 'Xdata', X)
                            set(lin(j,k), 'Ydata', Y)
                        else
                            hidem(lin(j,k))
                        end
                        
                        % Prepare for leader selection
                        if strcmp(lead_select,'popular')
                            const = ~cellfun(@isempty,c{i}.know(:,1));
                            pop_remain = zeros(size(const));
                            pop_remain(i) = sum(const)-1;
                            for loop = 1:length(const)
                                if const(loop) && loop ~= i
                                    pop_remain(loop) = sum(~cellfun(@isempty,c{i}.know{loop}.know(:,1))) - 1;
                                end
                            end
                            c{i}.election = pop_remain;
                        end
                        
                    end
                    
                end
                
            end
            
            % Plot the cars
            if c{i}.state(1) < c{i}.path(2)
                set(car{i}, 'Xdata', c{i}.state(1))
                set(car{i}, 'Ydata', c{i}.state(2))
                set(clabel{i}, 'Position', [c{i}.state(1)-2.5, c{i}.state(2)])
                set(clabel{i}, 'Color', [1 1 1] );
                set(clabel{i}, 'FontWeight', 'bold')
                set(clabel{i}, 'FontSize', 20)
                drawnow
            else
                for loop = 1:2
                    finished{finish_count} = c{i};
                end
                finish_count = finish_count + 1;
                delete(lin(i,:))
                delete(lin(:,i))
            end
            
        end
        
        M(iter) = getframe;
        
    end
    
    % Select the leader
    for i = 1:queue_pos
        if strcmp(lead_select,'popular')
            max_val     = max(c{i}.election);
            lead_idx    = find(max_val==c{i}.election);
            for loop = 1:2
                if ~isempty(lead_idx)
                    c{i}.leader = c{lead_idx};
                end
            end
        end
    end
    
    for i = 1:ncars
        car_pos(i) = c{i}.state(1);
    end
    min_pos = min(car_pos);
    
    % Increment timestep
    cur_time = cur_time + dt_sim;
    
    % Recording movie iteration
    iter = iter + 1;
    
end

% Print out who the leaders are
all_leaders = [];
for i = 1:ncars
    lead_belief = c{i}.leader.ID;
    lead_inArray = find(lead_belief == all_leaders);
    if isempty(lead_inArray)
        all_leaders = [all_leaders, lead_belief];
    end
end

fspec = repmat('%d ', 1, length(all_leaders));

fprintf('\nTotal time taken for first leg of system: %f\n\n', cur_time)
fprintf(['Selected leader(s) are: ', fspec,'\n\n'], all_leaders)


% Make movie
%
movie(M,1)
myVideo = VideoWriter('vid1.avi');
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);
%}