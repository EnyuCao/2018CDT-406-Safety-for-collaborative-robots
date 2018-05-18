function [pred] = predecessor(h)
    switch h
        case 1
            pred = 5;
        case 2
            pred = 6;
        case 3
            pred = 7;
        case 4 
            pred = 8;
        case 5 
            pred = 1;
        case 6 
            pred = 2;
        case 7
            pred = 3;
        case 8
            pred = 4;
    end
end