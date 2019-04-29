classdef carV1
    % This class contains all the properties and methods to operate on a
    % car object. Properties include physical attributes of the system,
    % including mass, collision sphere of influence, communication sphere
    % of influence, velocity limits, force limits, position and velocity
    % state, applied force, path
    
    % Physical properties
    properties (Access = public)
        ID          % ID of the car
        mass        % Mass of the car
        collSOI     % Collision sphere of influence
        commSOI     % Communication sphere of influence
        vLim        % Velocity limits (2x1 array)
        fLim        % Force limits (2x1 array)
        state       % State of the car [x y vx vy] (4x1 array)
        force       % Force (Fx, Fy) acting on car (2x1 array)
        path        % Input and output from the world (2x1 array)
        leader      % Flag that determines if car is leader
        inter       % Flag that determines if car is in or out of path (2x1 array)
    end
    
    methods
        
        % Constructor
        function obj = carV1(ID, mass, collSOI, commSOI, vLim, force, ...
                fLim, state, path, leader, inter)
            obj.ID = ID;
            obj.mass = mass;
            obj.collSOI = collSOI;
            obj.commSOI = commSOI;
            obj.vLim = vLim;
            obj.force = force;
            obj.fLim = fLim;
            obj.state = state;
            obj.path = path;
            obj.leader = leader;
            obj.inter = inter;
        end
        
        % Check to see if the state velocity exceeds limits
        function obj = velCheck(obj)
            if obj.state(3) >= obj.vLim(2)
                obj.force = [0, 0];
            end
        end
        
        % Check for intersection entering or leaving
        function obj = interSet(obj)
            if (obj.state(1) > obj.path(1) && obj.state(1) < obj.path(2))
                if (obj.inter(2) == 0)
                    obj.inter(2) = 1;
                elseif (obj.inter(2) == 1)
                    obj.inter(1) = 1;
                end
            else
                obj.inter(2) = 0;
            end
        end
        
        % Propagate the state at every time step
        function obj = statePropagate(obj,dt)
            dstate = [obj.state(3:4), ...
                obj.force(1)/obj.mass, obj.force(2)/obj.mass];
            obj.state = obj.state + dstate*dt;
        end
        
        % Check if an object collision occurs
        function obj1 = checkColl(obj1, obj2)
            if (obj1.state(3) > obj2.state(3)) && ...
                    (obj2.state(1)-obj1.state(1) <= obj1.collSOI+obj2.collSOI)
                obj1.state(3) = obj2.state(3);
                obj1.force(1) = 0;
            else
                obj1.force(1) = obj1.fLim(1);
            end
        end
        
    end
    
end

