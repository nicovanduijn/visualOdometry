function [ dist ] = epipolarDistance( F, p1, p2 )
%EPIPOLARDISTANCE Derived this using syms
%   should be very fast and correct, but only works on single point
dist=((F(3,3) + F(3,1)*p1(1) + F(3,2)*p1(2) + p2(1)*(F(1,3) + F(1,1)*p1(1) + F(1,2)*p1(2)) + p2(2)*(F(2,3) + F(2,1)*p1(1) + F(2,2)*p1(2)))^2/((F(1,3) + F(1,1)*p1(1) + F(1,2)*p1(2))^2 + (F(2,3) + F(2,1)*p1(1) + F(2,2)*p1(2))^2))^(1/2);
end

