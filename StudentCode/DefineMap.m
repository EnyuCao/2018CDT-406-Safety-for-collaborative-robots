function [map] = DefineMap(robxy)

map = zeros(12,12,3);%Initialize mapmatrix

%define outer boundaries 
map(:,1,1) = 3;
map(:,12,1) = 3;
map(1,:,1) = 3;
map(12,:,1) = 3;

%Convert robot position into matrix entries
i  = floor((2-robxy(2))/0.5);
j  = floor((2+robxy(1))/0.5);

map(i,j,1) = 2;%Update matrix with robot position
end