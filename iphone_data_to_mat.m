function [  ] = iphone_data_to_mat(  )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 
%% LOAD ACCEL DATA
 
%% Initialize variables.
filename = '/Users/kunalmenda/Google Drive/2016 Spring/E160/E160/Lab2/IPHONE_DATA/accelerometer.csv';
delimiter = ',';
startRow = 2;
 
%% Format string for each line of text:
%   column1: double (%f)
%   column2: double (%f)
%   column3: double (%f)
%   column4: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%[^\n\r]';
 
%% Open the text file.
fileID = fopen(filename,'r');
 
%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
 
%% Close the text file.
fclose(fileID);
 
%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.
 
%% Allocate imported array to column variable names
timestamp = dataArray{:, 1};
xinGs = dataArray{:, 2};
yinGs = dataArray{:, 3};
zinGs = dataArray{:, 4};
 
 
%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;
 
 
%% LOAD GYRO DATA
 
%% Initialize variables.
filename = '/Users/kunalmenda/Google Drive/2016 Spring/E160/E160/Lab2/IPHONE_DATA/gyroscope.csv';
delimiter = ',';
startRow = 2;
 
%% Format string for each line of text:
%   column1: double (%f)
%   column2: double (%f)
%   column3: double (%f)
%   column4: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%[^\n\r]';
 
%% Open the text file.
fileID = fopen(filename,'r');
 
%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
 
%% Close the text file.
fclose(fileID);
 
%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.
 
%% Allocate imported array to column variable names
timestamp1 = dataArray{:, 1};
xinradianss = dataArray{:, 2};
yinradianss = dataArray{:, 3};
zinradianss = dataArray{:, 4};
 
 
%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;
 
%% PROCESS DATA
 
% Normalize timestamps
timestamp = timestamp - timestamp(1);
timestamp1 = timestamp1 - timestamp1(1);
 
% Interpolate gyro data
 
x_rads = interp1(timestamp1, xinradianss, timestamp);
y_rads = interp1(timestamp1, yinradianss, timestamp);
z_rads = interp1(timestamp1, zinradianss, timestamp);
 
data = [timestamp, [xinGs,yinGs,zinGs]*9.81, x_rads,y_rads,z_rads];
initial = [0,0,0; 0,0,0; 0,0,0];
 
 
% Subtract SS error from first 5 seconds of data
% data_to_avg = data(data(:,1)<5,2:end);
% ss_error = nanmean(data_to_avg);
% ss_error = [0,ss_error];
% [m,~] = size(data);
% ss_error = repmat(ss_error,m,1);
% data = data - ss_error;
 
save data.mat data initial
 
 
end
 


