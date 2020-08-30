parfor i = 1:2
    if i == 1
      disp('hello')
    else
      disp('bye')
    end
end
%%
b1 = [1,-0.4,0.03]'; 
b2 = [-1,-0.15,0.03]';
b3 = [1,0.1,0.03]';
b4 = [-1,0.35,0.03]'; 
b5 = [-1,0.6,0.03]';
b6 = [1,0.85,0.03]';
b7 = [-1,0.2,-0.2]';
b8 = [1,-0.05,-0.2]';
b9 = [-1,-0.3,-0.2]';
bricks = [b1 b2 b3 b4 b5 b6 b7 b8 b9]'

for i=1:length(bricks)
    brickadjusted = min(