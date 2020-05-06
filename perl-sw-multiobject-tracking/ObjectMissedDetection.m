function undetected_object=ObjectMissedDetection(object)
% Function used to update state estimates with missed detections
            %
            %   Inputs:
            %       object      - undetected object predicted estimate
            %
            %   Outputs:
            %       undetected_object - undetected object updated estimate
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
    undetected_object.weight=0.02*object.weight;
    undetected_object.mu=object.mu;
    undetected_object.P=object.P;
end