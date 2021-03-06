%% Import data from text file.
% Script for importing data from the following text file:
%
%    /Users/kunalmenda/Google Drive/2016 Spring/E160/E160/Jaguar_BaseCode_03_alt/bin/x86/Release/JaguarData_2016-2-25-37.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2016/02/25 17:39:23

%% Initialize variables.
filename = '/Users/kunalmenda/Google Drive/2016 Spring/E160/E160/Jaguar_BaseCode_03_alt/bin/x86/Release/JaguarData_2016-2-25-30.txt';
delimiter = ' ';

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
time = dataArray{:, 1};
desL = dataArray{:, 2};
actL = dataArray{:, 3};
desR = dataArray{:, 4};
actR = dataArray{:, 5};


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;


Ts = mean(diff(time));
tt = [0:Ts:time(end)]';
desL = interp1(time,desL,tt);
desL(1) = 0;
actL = interp1(time,actL,tt);
actL(1) = 0;
desR = interp1(time,desR,tt);
desR(1) = 0;
actR = interp1(time,actR,tt);
actR(1) = 0;

figure();
plot(tt,[desL,actL]);
grid on
figure();
plot(tt,[desR,actR]);
grid on;


