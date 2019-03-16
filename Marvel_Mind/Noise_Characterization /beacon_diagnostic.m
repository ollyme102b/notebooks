% script to diagnose any beacons that are noisier than others
close all;
%% Import Data
%data_file = 'Data/WithBackgroundNoise.log';
data_file = 'Data/NoBackgroundNoise.log';
[position, distance_signals] = import_data(data_file);

%% Plot Positions
figure;
plot(position);
legend('x','y','z','location','best');

%% Plot Distances
figure; hold on;
labels = {};
for i = 1:length(distance_signals)
    plot(distance_signals{i});
    labels = {labels{:},distance_signals{i}.name};
end
legend(labels,'location','best');

%% Aux Functions
function [position_ts, signal_data] = import_data(file_path)

    csv_data = csvread(file_path); % read data into matrix
    n_cols = size(csv_data,2); % number of data columns
    
    time = csv_data(:,1)/1000; % [s]
    time = time - time(1); % start time at 0
    x_data = csv_data(:,5); % [m]
    y_data = csv_data(:,6); % [m]
    z_data = csv_data(:,7); % [m]
    
    position_ts = timeseries([x_data,y_data,z_data],time,...
        'name','position'); % create time series
    position_ts.DataInfo.Units = 'm'; % add unit to time series
    
    % assemble cell array of time series of signals
    signal_data = {};
    for c = n_cols-8:2:n_cols-1
        beacon_number = csv_data(1,c);
        distances = csv_data(:,c+1);
        
        ts = timeseries(distances,time,...
            'name',['Beacon ',num2str(beacon_number)]);
        ts.DataInfo.Units = 's';
        
        signal_data = {signal_data{:},ts};
    end
end