function [ output ] = homogenize( input )
%HOMOGENIZE homogenizes n number of m-dimensional points given as mxn
%matrix
%   Detailed explanation goes here

output = [input; ones(1,size(input,2))];
end

