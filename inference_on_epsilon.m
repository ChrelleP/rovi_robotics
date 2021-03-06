%% Robotics Mandatory 2
% Statistic inference on epsilon value for RRT based method
clc; clear;

% Variables used for loading data
basename = '0000_eps.txt';
folder = './data/';
delim = '\t';
header = 1;

% Data matrices
length_matrix = [];
config_matrix = [];
time_matrix = [];

for i = 0.05:0.05:2.0
    % Generate filename and load data
    sample = num2str(i);
    filename = strcat(folder, sample, basename);
    file_data = importdata(filename, delim, header);
    
    % Split up data into length and time
    data = file_data.data;
    
    length_data = data(:,1);
    time_data = data(:,3);
    config_data = data(:,2);
    
    % Append to the data matrices
    length_matrix   = [length_matrix, length_data];
    time_matrix     = [time_matrix, time_data];
    config_matrix   = [config_matrix, config_data];
end

% Calculate mu, sigma and CI for euclidean length, time and configuration space length. 
mu_length       = nanmean(length_matrix);
sigma_length    = nanvar(length_matrix); 
CI_length       = transpose(zeros(2, length(mu_length)));
err_length      = transpose(zeros(1, length(mu_length)));

mu_config         = nanmean(config_matrix);
sigma_config      = nanvar(config_matrix);
CI_config         = transpose(zeros(2, length(mu_config)));
err_config        = transpose(zeros(1, length(mu_config)));

mu_time         = nanmean(time_matrix);
sigma_time      = nanvar(time_matrix);
CI_time         = transpose(zeros(2, length(mu_time)));
err_time        = transpose(zeros(1, length(mu_time)));

for i = 1:length(CI_time)
    CI_length(i,1) = mu_length(i) - (1.96)*(sqrt(sigma_length(i)));
    CI_length(i,2) = mu_length(i) + (1.96)*(sqrt(sigma_length(i)));
    err_length(i)  = (1.96)*(sqrt(sigma_length(i)));
    
    CI_config(i,1) = mu_config(i) - (1.96)*(sqrt(sigma_config(i)));
    CI_config(i,2) = mu_config(i) + (1.96)*(sqrt(sigma_config(i)));
    err_config(i)  = (1.96)*(sqrt(sigma_config(i)));  
    
    CI_time(i,1) = mu_time(i) - (1.96)*(sqrt(sigma_time(i)));
    CI_time(i,2) = mu_time(i) + (1.96)*(sqrt(sigma_time(i)));
    err_time(i)  = (1.96)*(sqrt(sigma_time(i)));  
end


y = 0.05:0.05:0.95;

figure(1)
 plot(y, mu_length); hold on;
 errorbar(y, mu_length, err_length, '.')
 title('Comparison of means for euclidean lengths');
 xlabel('Epsilon')
 ylabel('Length')
    
figure(2) 
 plot(y, mu_config); hold on;
 errorbar(y, mu_config, err_config, '.');
 title('Comparison of means for summed joint movement');
 xlabel('Epsilon')
 ylabel('Summed Joint Movement')

figure(3)
 plot(y, mu_time); hold on;
 errorbar(y, mu_time, err_time, '.');
 title('Comparison of means for execution time');
 xlabel('Epsilon')
 ylabel('Time')

% Clear variables for clearer view
clearvars basename data file_data filename folder header i delim sample y;