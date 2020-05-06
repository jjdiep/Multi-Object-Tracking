function pred_object=ExistingObjectMotionModel(object)
% Function that propogates existing state estimates
%
            %   Inputs:
            %       object      - Exisitng state estimate
            %
            %   Outputs:
            %       pred_object - Predicted state estimate
            
    F=eye(3);
    Q=25*[0.25*eye(2) 0.5];
    pred_object.weight=0.99*object.weight;
    pred_object.mu=F*object.mu;
    pred_object.P=Q+F*object.P*F';
end 