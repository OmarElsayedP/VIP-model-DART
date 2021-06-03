%Create n normal footsteps
function fs_plan = varstep(n)
fs_plan = zeros(n,2);
for i = 2:n
    if(mod(i,2))
        fs_plan(i,2) = 0.075;
    else
        fs_plan(i,2) = -0.075;
    end
    if(i > 2)
        fs_plan(i,1) = fs_plan(i-1,1) + 0.05;
    end
end
end
