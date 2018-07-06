function [myrand] = vec_rand_ab_res(vec_ab_res)
%MY_RAND_AB_RES Summary of this function goes here
%   a < random number < b and resolution

r = (vec_ab_res(2)-vec_ab_res(1))*rand(1);
ares = 1/vec_ab_res(3);
r = floor((vec_ab_res(3)/2)+r*ares);
myrand = vec_ab_res(1)+ r*vec_ab_res(3);
end

