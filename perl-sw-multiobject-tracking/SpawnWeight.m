function spawn_weight=SpawnWeight(spawn_x,parent_x)
% Function to return the spawned particle weight
            %
            %   Inputs:
            %       spawn_x      - spawned object state
            %
            %   Outputs:
            %       spawn_weight - weight assigned to the spawned object
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
Q=diag([10,10,40]');
spawn_weight=0.05*mvnpdf(spawn_x,parent_x,Q);
end