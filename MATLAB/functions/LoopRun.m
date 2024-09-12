function P_loop = LoopRun(P,times)
    P_loop = [];
    for i=1:times
        P_loop = [P_loop; P];
    end
end