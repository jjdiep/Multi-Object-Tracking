function z= RCNNSensor(t)
    tracklets = open("tracklets.mat");
    tracklets = tracklets.tracklets;
    i = 1 ;
    for obj_idx=1:numel(tracklets{t})
        if(~strcmp(tracklets{t}(obj_idx).type, 'DontCare'))
            z(i,:) = tracklets{t}(obj_idx).t;
            i = i+1;
        end
    end
end