function [wanted] = FindNextPos(i,j,travel2,map,gi,gj)
% returns the angle that the robot needs to rotate towards
    wanted = -4;
    %ap = adjacent position
    ap(1,:) = [i-1,j];
    ap(2,:) = [i-1,j+1];
    ap(3,:) = [i,j+1];
    ap(4,:) = [i+1,j+1];
    ap(5,:) = [i+1,j];
    ap(6,:) = [i+1,j-1];
    ap(7,:) = [i,j-1];
    ap(8,:) = [i-1,j-1];

    for h = 1:9
        switch h   
            case 1
                if map(ap(h,1),ap(h,2),3) == 5 && ismember(h,travel2) == 1
                   wanted = 1.57;
                end
            case 2
                if map(ap(h,1),ap(h,2),3) == 6 && ismember(h,travel2) == 1
                   wanted = 0.785;
                end
            case 3
                if map(ap(h,1),ap(h,2),3) == 7 && ismember(h,travel2) == 1
                   wanted = 0;
                end
            case 4
                if map(ap(h,1),ap(h,2),3) == 8 && ismember(h,travel2) == 1
                   wanted = -0.785;
                end
            case 5
                if map(ap(h,1),ap(h,2),3) == 1 && ismember(h,travel2) == 1
                   wanted = -1.57;
                end
            case 6
                if map(ap(h,1),ap(h,2),3) == 2 && ismember(h,travel2) == 1
                   wanted = -2.355;
                end
            case 7
                if map(ap(h,1),ap(h,2),3) == 3 && ismember(h,travel2) == 1
                   wanted = 3.14;
                end
            case 8
                if map(ap(h,1),ap(h,2),3) == 4 && ismember(h,travel2) == 1
                   wanted = 2.355;
                end
            case 9
                if gi == i && gj == j
                   wanted = -5;
                end 
        end 
    end
end