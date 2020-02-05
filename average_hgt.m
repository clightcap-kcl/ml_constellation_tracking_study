function [ mat ] = average_hgt( matCloud )
%AVERAGE_hgt Takes N 4x4 homogeneous matrices as a 4x4xN matrix and
%averages them.
%

mat = eye(4);
R = sum(matCloud(1:3,1:3,:),3);
T = mean(matCloud(1:3,4,:),3);
[U,~,V] = svd(R);

mat(1:3,1:3) = U*V';
mat(1:3,4) = T;