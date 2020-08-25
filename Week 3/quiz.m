clc;
close all;
clear;

mdl_puma560;
p560.teach;

%%

angle = deg2rad(35);

max_lim = p560.qlim(:,2);
min_lim = p560.qlim(:,1);

%%
res = [];

for i = 1:size(max_lim,1)
    diff_max = abs(max_lim(i) - angle);
    diff_min = abs(min_lim(i) - angle);
    
    if diff_max <= diff_min
        res(i) = diff_max;
    else
        res(i) = diff_min;
    end
end

res

%%
mov = transl(90,0,0) * trotz(pi/4);
cpos=se3(se2(0,100,0));
iterations = 3;

for i=1:iterations
    cpos = cpos * mov;
end

cpos