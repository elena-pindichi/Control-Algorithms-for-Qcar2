clc, close all;

load('trajectory.mat');
t = 0:0.1:50;
ref = [tt', xref', yref', dddxref', dddyref', ddxref', ddyref', dxref', dyref'];
ref = timeseries(ref);