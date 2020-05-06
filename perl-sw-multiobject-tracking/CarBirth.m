function birth_objects= CarBirth()
% Function creates random new born cars in the scene
%
            %   Inputs:
            %       x            - current state estimate
            %
            %   Outputs:
            %       birth_weight - weight assigned to newborn entities
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
    x=-12+rand(1,25)*20;
    y=-12+rand(1,25)*20;
    z=-50+rand(1,25)*100; 
    for i=1:25
        birth_objects(i).mu=[x(i);y(i);z(i)];
        birth_objects(i).P=diag([2,2,5]');
        birth_objects(i).weight=0.05;
    end
end