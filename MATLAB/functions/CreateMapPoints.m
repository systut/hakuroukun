function [P, point_num] = CreateMapPoints(length, width, init_position)
    % Parameters :
    w_turning_min = 4;
    n_width = 1.5;
    
    % Calculate number of coverage lines :
    div_check = mod(width,n_width);
    if div_check == 0
        n = width/n_width;
    else
        n = width/n_width + 1;
    end

    % The case that N >= 5
    % Robot initial position
    p_0 = init_position;

    % Generate all trajectory points on the map base on area
    for i = 0:n+1
        p_lower(i+1,:) = p_0 + [-i*n_width 0];
        p_middle(i+1,:) = p_0 + [-i*n_width length/2];  
        p_upper(i+1,:) = p_0 + [-i*n_width length];
    end

    % Generate trajectory 
    % NOTE : n_width must be smaller or equal to w_turning_min
    n_upper_turn = ceil(w_turning_min/n_width);  
    % n_upper_turn = n_per_turn + 1;
    n_turn = n - n_upper_turn;
    disp(n_turn)
    % Output
    P = [];
    for k=1:n_turn+1
        P = [P; p_lower(k,:); 
                % p_middle(k,:); 
                p_upper(k,:); 
                p_upper(k+n_upper_turn,:); 
                % p_middle(k+n_upper_turn,:);
                p_lower(k+n_upper_turn,:)];
    end
    point_num = size(P,1);
end