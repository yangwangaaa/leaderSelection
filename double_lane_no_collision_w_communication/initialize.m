%% MATLAB initialization
clc
clear
close all

% Create a random seed
s = rng;

% System properties
ncars       = 10;
nlane       = 3;
path        = [0, 100];

% Simulation integrator time step
tol         = 2;
dt_sim      = 5*10^-tol;

% Lane selection algorithms
lane_algo = {'greedy','v1','v2'};

%% Car initialization
% Create car queue

% Specified conditions
%{
collSOI     = {5,5,5,5,5};
commSOI     = {20,20,20,20,20};
vLim        = {15,30,60,10,20};
aLim        = {5,10,5,30,20};
accel       = {10,10,10,10,10};
state       = {[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]};
leader      = 0;
know        = cell(ncars,1);
election    = 0;
c           = cell(ncars,1);
for i = 1:ncars
    c{i}    = carV2( i, collSOI{i}, commSOI{i}, vLim{i}, ...
        aLim{i}, accel{i}, state{i}, path, leader, know, election );
end
%}

for run = 1:3
    
    % Get the same seed as before
    rng(s)
    
    % Create the cars
    c = cell(ncars,1);
    for i = 1:ncars
        collSOI = 2;
        commSOI = 5*collSOI;
        vLim    = 5*randi(5);
        aLim    = 5*randi(5);
        accel   = 0;
        state   = [0,0,0,0];
        leader	= 0;
        know    = cell(ncars,1);
        election= 0;
        c{i}    = carV2( i, collSOI, commSOI, vLim, aLim, accel, ...
            state, path, leader, know, election );
    end
    
    % Run the algorithm
    lane_selection = lane_algo{run};
    runV2
    pause
    close all
    
end