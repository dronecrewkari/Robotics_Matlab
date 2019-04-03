clc
close all;
clear;
% addpath(genpath(pwd))
SetMode(2);
global model;  % model 1 simulation, model 2 real data
if (model == 1)
%     run("multirobot_simulation.m");
%     run("createPlot.m");
elseif (model == 2)
    run('dataset\MRCLAM_Dataset3\loadMRCLAMdataSet.m')
    run('dataset\MRCLAM_Dataset3\sampleMRCLAMdataSet.m')
    run('sample1.m')
    run('configuration.m')
    run('project\Multirobot_localization.m')
    run('result\Animate.m')
elseif (model == 3)
    run('dataset\Dataset_IROS\sample2.m');
    run('configuration.m')
    run('project\Multirobot_localization.m')
    run('result\Animate.m')
end


