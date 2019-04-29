classdef carV2 < handle
    % This class contains all the properties and methods to operate on a
    % car object. Properties include physical attributes of the system,
    % including mass, collision sphere of influence, communication sphere
    % of influence, velocity limits, force limits, position and velocity
    % state, applied force, path, leader, and knowledge.
    
    % Physical properties
    properties (Access = public)
        ID          % Number of car in original queue
        collSOI     % Collision sphere of influence
        commSOI     % Communication sphere of influence
        vLim        % Velocity limit
        aLim        % Acceleration limit
        state       % State of the car [x y vx vy] (4x1 array)
        accel       % Acceleration acting on car
        path        % Input and output from the world (2x1 array)
        leader      % Flag that determines if car is leader
        know        % Knowledge of other cars
        election    % Flag for whether or not agent partakes in election
    end
    
    methods
        
        % Constructor
        function obj = carV2(ID, collSOI, commSOI, vLim, aLim, ...
                accel, state, path, leader, know, election)
            obj.ID      = ID;
            obj.collSOI = collSOI;
            obj.commSOI = commSOI;
            obj.vLim    = vLim;
            obj.accel   = accel;
            obj.aLim    = aLim;
            obj.state   = state;
            obj.path    = path;
            obj.leader  = leader;
            obj.know    = know;
            obj.election= election;
        end
        
    end
    
end

