function [P, point_num] = CreateMapPoints(length, width, n_width)
    minimum_turning_width = 3;
    div_check = mod(width,n_width);
    if div_check == 0
        n = width/n_width;
    else
        n = width/n_width + 1;
    end

    P_0 = [0 0];
    for i = 0:n
        P_under_n = P_0 + [-i*n_width 0];
        P_mid_n = P_0 + [-i*n_width length/2];
        P_upper_n = P_0 + [-i*n_width length];
        P_under(i+1,:) = P_under_n;
        P_mid(i+1,:) = P_mid_n;
        P_upper(i+1,:) = P_upper_n;
    end

    n_per_turn = fix(minimum_turning_width/n_width);
    n_upper_turn = n_per_turn + 1;
    n_turn = n - n_upper_turn;

    % Output
    P = [];
    for k=1:n_turn+1
        P = [P; P_under(k,:); P_mid(k,:); P_upper(k,:); 
                P_upper(k+n_upper_turn,:); P_mid(k+n_upper_turn,:);
                P_under(k+n_upper_turn,:)];
    end
    point_num = size(P,1);
end