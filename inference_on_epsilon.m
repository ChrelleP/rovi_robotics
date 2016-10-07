%% Robotics Mandatory 2
% Statistic inference on epsilon value for RRT based method

% Variables used for loading data
basename = '00000_eps.txt';
folder = './data/';
delim = '\t';
header = 1;

% Data matrices
length_matrix = [];
time_matrix = [];

for i=0.1:0.1:0.9
    % Generate filename and load data
    sample = num2str(i);
    filename = strcat(folder, sample, basename);
    file_data = importdata(filename, delim, header);
    
    % Split up data into length and time
    data = file_data.data;
    
    length_data = data(:,1);
    time_data = data(:,2);
    
    % Append to the data matrices
    length_matrix   = [length_matrix, length_data];
    time_matrix     = [time_matrix, time_data];
end

% Calculate mu and sigma for length and time
mu_length       = nanmean(length_matrix);
sigma_length    = nanvar(length_matrix); 

mu_time         = nanmean(time_matrix);
sigma_time      = nanvar(time_matrix);

% Clear variables for clearer view
clearvars -except mu_length mu_time sigma_length sigma_time length_matrix time_matrix;