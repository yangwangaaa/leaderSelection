%% MATLAB initialization
clc
clear
close all

% Create a random seed
s = rng;

% System properties
ncars       = 5;
lanes       = [1,2,3,4,5];
path        = [0, 100, 200];

% Simulation integrator time step
tol         = 2;
dt_sim      = 5*10^-tol;

% Lane selection algorithms
lane_algorithm      = {'greedy'};%,'v1','v2'};

% Leader selection criteria
leader_selection    = {'popular',''};

%% Car initialization
% Create car queue

for run = 2:length(lanes)
    
    nlane = lanes(run);
    
    % Get the same seed as before
    % rng(s)
    
    % Create the cars
    %{
    c = cell(ncars,1);
    for i = 1:ncars
        collSOI = 5;
        commSOI = 5*collSOI;
        vLim    = 5*randi(5);
        aLim    = 10;
        accel   = 0;
        state   = [0,0,0,0];
        leader	= cell(1,1);
        know    = cell(ncars,2);
        election= [];
        c{i}    = carV2( i, collSOI, commSOI, vLim, aLim, accel, ...
            state, path, leader, know, election );
        for loop = 1:2
            c{i}.know{i} = c{i};
        end
    end
    %}
    
    % Specified conditions
    %
    collSOI     = {5,5,5,5,5};
    commSOI     = {30,30,30,30,30};
    vLim        = {5,10,15,20,25};
    aLim        = {10,10,10,10,10};
    accel       = {0,0,0,0,0};
    state       = {[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]};
    leader      = cell(1,1);
    know        = cell(ncars,2);
    election    = [];
    c           = cell(ncars,1);
    for i = 1:ncars
        c{i}    = carV2( i, collSOI{i}, commSOI{i}, vLim{i}, ...
            aLim{i}, accel{i}, state{i}, path, leader, know, election );
        for loop = 1:2
            c{i}.know{i} = c{i};
        end
    end
    %}
    
    % Run the algorithm
    lane_selection  = 'greedy';
    lead_select     = 'popular';
    runV2
    pause
    clear M myVideo
    
end