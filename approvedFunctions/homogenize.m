function [ output] = homogenize( input )
%HOMOGENIZE homogenizes points
%   adds a row of ones to the bottom
output=[input;ones(1,size(input,2))];
end

