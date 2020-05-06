function spawn_object=SpawnMotionModel(object)
% Function used create and propogate spawned estimates
            %
            %   Inputs:
            %       object     - object consisting of parent state estimate
            %
            %   Outputs:
            %       spwan_object - object consisting of a spawned estimate
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
    spawn_object.mu=(mvnrnd(zeros(3,1),diag([5,5,10]')))'+object.mu;
    spawn_object.P=diag([5,5,10]')+object.P;
    spawn_object.weight=0;
end