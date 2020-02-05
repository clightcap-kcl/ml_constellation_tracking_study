function plot_directions(vectors)

% function will plot list of direction vectors
linespec = {'b','r','k','g','m','c'};
num_linespecs = length(linespec);

figure, hold on

num_vectors = length(vectors);
for i=1:num_vectors
    vec_label = rem(i-1,floor(num_vectors/2))+1;
    vec = vectors(i,:);
    quiver3(0,0,0,vec(1),vec(2),vec(3),0,linespec{rem(vec_label,num_linespecs)+1});
    text(vec(1),vec(2),vec(3),strcat('\leftarrow',num2str(vec_label,'%2.0f')));
end
axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Rotational Directions');
set(gca,'FontSize',15);


