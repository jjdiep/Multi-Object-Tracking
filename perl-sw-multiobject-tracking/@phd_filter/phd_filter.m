classdef phd_filter < handle
    % PHD filter for multitarget tracking
    
    % PUBLIC PROPERTIES
    properties
        % Known amount of objects at time step 0
        J_k;
        % Number ofw objects spawned from  existing objects every time step
        J_beta;
        % Number of objects born at birth positions every time step
        J_gamma; 
        % Std Deviation associated with the Motion Model 
        sigma_v;
        % Probability of survival
        p_s;
        % Probability of detection
        p_d;
        % Merging Threshold in terms of weights
        T;
        % Merging Threshold in terms of Mahalanobis distance
        U;
        % Maximum amounti of allowable objects @ each time step after
        % Pruning and Merging
        J_max;
        % Dynamic Count for number of objects
        i;
        % Locations at which objects are born with zero vecloity
        mu_gamma;
        %Number of time steps
        t_steps;
        %
        x_k;
        
    end
    
    % PRIVATE PROPERTIES
    properties(Access = private)
        % List of objects consisting of updated state estimates
        %x_k;
        % List of objects consisting of predicted state estimates
        x_pred;
        % Sensor Reading
        z;
        % Ground Truth
        g;
        % Time step
        t;
        % Motion Model
        F;
        % Sensor Model
        H;
        % Optical Flow estimates for each class objet in diagonal matrix
        % formm
        flow;
        % Optical flow lst for all i11mages
        flow_list;
        % Bounding boxes
        bbox;
        % List of PreCalculated update parameters for the Filter
        PHDupdate;
        % List of trajectories for every object in the scene
        traj;
        % List of Ground truth trajectories
        ground_truth;
        % Type of object
        type;
    end
    
    % PRIVATE CONSTANTS
    properties(Access = private, Constant)
        % Covariance used to for object initialization at time step 0
        init_covP=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
        % Covariance associated with weighting new born objects
        P_gamma=diag([100,100,25,25]');
        % Additional covariance associated with spawned objects
        P_beta=diag([50,50,10,10]');
        % Covariance associated with weighting spawned objects
        weight_beta_P=diag([100,100,400,400]');
        % Existing Object Motion Noise
        Q=5*[0.25*eye(2) 0.5*eye(2);0.5*eye(2) eye(2)];
        % PHD update Noise
        R=100*eye(2);
    end
    
    methods
        % Class constructor
        function obj = phd_filter(class_name)
            if nargin== 0
                class_name="simulation";
            end
            if(class_name=="simulation")
                obj.type=0;
                obj.J_k=2;
                obj.J_beta=2;
                obj.J_gamma=2;
                obj.sigma_v=5;
                obj.p_s=0.99;
                obj.p_d=0.98;
                obj.T=0.00001;
                obj.U=4;
                obj.J_max=100;
                obj.i=0;
                obj.mu_gamma=[[250,250,0,0]',[-250,-250,0,0]'];
                obj.t_steps=100;
                obj.x_k(1).mu=[250,250,0,0]';
                obj.x_k(2).mu=[-250,-250,0,0]';
                obj.x_k(1).P=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
                obj.x_k(2).P=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
                obj.x_k(1).weight=0.5;
                obj.x_k(2).weight=0.5;
                obj.F=[eye(2) eye(2);zeros(2) eye(2)];
                obj.H=[eye(2) zeros(2)];
                obj.PHDupdate=struct('eta',{},'K',{},'S',{});
                
                
            end
            if(class_name=="car")
                obj.type=1;
                obj.J_k=2;
                obj.J_beta=2;
                obj.J_gamma=2;
                obj.sigma_v=5;
                obj.p_s=0.99;
                obj.p_d=0.98;
                obj.T=0.00001;
                obj.U=6;
                obj.J_max=100;
                obj.i=0;
                obj.mu_gamma=[[250,250,0,0]',[-250,-250,0,0]'];
                obj.t_steps=41;
                obj.x_k(1).mu=[550,520,0,0]';
                obj.x_k(2).mu=[1134,496,0,0]';
                obj.x_k(3).mu=[905,462,0,0]';
                obj.x_k(1).P=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
                obj.x_k(2).P=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
                obj.x_k(3).P=[100 0 0 0;0 100 0 0;0 0 10 0;0 0 0 10];
                obj.x_k(1).weight=0.33;
                obj.x_k(2).weight=0.33;
                obj.x_k(3).weight=0.33;
                obj.flow_list=open("Test/flow.mat");
                obj.flow=eye(2);
                obj.F=[eye(2) eye(2);zeros(2) obj.flow];
                obj.H=[eye(2) zeros(2)];
                obj.bbox=open("Test/BBox/BBoxData.mat");
                obj.bbox=struct2cell(obj.bbox);
                obj.PHDupdate=struct('eta',{},'K',{},'S',{});
            end
        end
        %
        function birth_weight=BirthWeight(obj,x)
            % Function returns the value of the weights of the new born entities at
            % [250,250] & [-250,-250]
            %
            %   Inputs:
            %       x            - current state estimate
            %
            %   Outputs:
            %       birth_weight - weight assigned to newborn entities
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
            birth_weight=0.1*mvnpdf(x,obj.mu_gamma(1),obj.P_gamma)+0.1*mvnpdf(x,obj.mu_gamma(2),obj.P_gamma);
        end
        function birth_objects= CarBirth(obj)
        % Function creates random new born cars in the scene
        %
        %   Outputs:
                    %   birth_objects - Randomly generated objects
                    %
                    %   Author: Sidddharth Ratapani Navin
                    %   Date:   11/23/2018
            x=256+rand(1,50)*768;
            y=rand(1,50)*2048;
            for iter=1:50
                birth_objects(iter).mu=[x(iter);y(iter);0;0];
                birth_objects(iter).P=diag([100,100,25,25]');
                birth_objects(iter).weight=0.05;
            end
            obj.i=50;
        end
        function spawn_weight=SpawnWeight(obj,spawn_x,parent_x)
            % Function to return the spawned particle weight
            %
            %   Inputs:
            %       spawn_x      - spawned object state
            %       parent_x     - parent object of spawned object
            %
            %   Outputs:
            %       spawn_weight - weight assigned to the spawned object
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
            spawn_weight=0.05*mvnpdf(spawn_x,parent_x,obj.weight_beta_P);
        end
        %
        function spawn_target=SpawnMotionModel(obj, spawned_target)
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
            spawn_target.mu=(mvnrnd(zeros(4,1),diag([50,50,10,10]')))'+spawned_target.mu;
            spawn_target.P=obj.P_beta+object.P;
            spawn_target.weight=0;
        end
        %
        function pred_target=ExistingObjectMotionModel(obj, existing_target)
            % Function that propogates existing state estimates
%
            %   Inputs:
            %       target      - Exisitng state estimate
            %
            %   Outputs:
            %       pred_target - Predicted state estimate
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
            pred_target.weight=obj.p_s*existing_target.weight;
            pred_target.mu=obj.F*existing_target.mu;
            pred_target.P=obj.Q+obj.F*existing_target.P*obj.F';
            
        end
        %
        function [tracked_PHDupdate,tracked_P]= UpdatePHDComponents(obj,tracked_target)
            % Function that calculates the values of the PHD update paramters for each of the
            % predicted estimates
            %
            %   Inputs:
            %       tracked_target    - Predicted object state estimate
            %
            %   Outputs:
            %       tracked_PHDupdate - Pre-calculated Update Components
            %       tracked_P         - Updated Covariance estimate  
            %
            %   Author: Siddharth Ratapani Navin
            %   Date:   10/28/2018
            %% Calculate PHD Update Components            
                tracked_PHDupdate.eta=obj.H*tracked_target.mu;
                tracked_PHDupdate.S=obj.R+obj.H*tracked_target.P*obj.H';
                tracked_PHDupdate.S=(tracked_PHDupdate.S+tracked_PHDupdate.S')/2; % hack because MATLAB thinks it is not symmtric because of digit precision
                tracked_PHDupdate.K = tracked_target.P * obj.H' * (tracked_PHDupdate.S \ eye(size(tracked_PHDupdate.S))); 
            %% Update Covariance
                I = eye(size(object.P));
                tracked_P = (I - tracked_PHDupdate.K * obj.H) * object.P * (I - tracked_PHDupdate.K * obj.H)' ...
                    + tracked_PHDupdate.K * obj.R * tracked_PHDupdate.K'; % Joseph update form
        end
        %
        function undetected_object=ObjectMissedDetection(obj, missed_object)
            % Function used to update state estimates with missed detections
            %
            %   Inputs:
            %       missed_object      - undetected object predicted estimate
            %
            %   Outputs:
            %       undetected_object  - undetected object updated estimate
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
            undetected_object.weight=(1-obj.p_d)*missed_object.weight;
            undetected_object.mu=missed_object.mu;
            undetected_object.P=missed_object.P;
        end
        %
        function  detected_object=DetectedObjectUpdate(obj,detected_object,z,detected_object_PHDupdate)
            % Function used to Update the state estimate with sensor readings  
            %
            %   Inputs:
            %       object      - Object for one single particle
            %       z           - Sensor reading
            %       PHDUpdate   - Object conssting of precalculated
            %                     parameters
            %       
            %   Outputs:,
            %       detected_object  - Object for one single particle with
            %                          updated state estimates
            %   Author: Siddharth Ratapani Navin
            %   Date:   10/28/2018
            detected_object.weight=obj.p_d*detected_object.weight*mvnpdf(z',detected_object_PHDupdate.eta,detected_object_PHDupdate.S);
            detected_object.mu=detected_object.mu+detected_object_PHDupdate.K*(z'-detected_object_PHDupdate.eta);
            detected_object.P=detected_object.P;
        end
        %
        function positionSensor(obj)
            % Function used to simulate sensor readings at timestep 'k'
            % Particles are spawned at [250,250] & [-250,-250] and die at [500,-1000] &
            % [1000,-500] after 100s. They meet at 50 s.
            %  A third particle is spawned after 66s at [575,-415] and dies at 100s.
            %
            %   Inputs:
            %       t     - current timestep 's'
            %
            %   Outputs:
            %       z     - Sensor reading at time 't' s
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
            obj.z(1,:)=mvnrnd([-250+12.7475*obj.t*0.9806;-250+12.7475*obj.t*-0.1961],10*eye(2)); % Sample sensor readings from the normal distrubtion with sigma=10 m/s^2
            obj.z(2,:)=mvnrnd([250+12.7475*obj.t*0.1961;250+12.7475*obj.t*-0.9806],10*eye(2));
            if(obj.t>66) % Spsawned Particle Sensing
                obj.z(3,:)=mvnrnd([417.4+12.7475*(obj.t-67)*-0.92;-587+12.7475*(obj.t-67)*-0.39],10*eye(2));
            end   
        end
        function groundTruth(obj)
            % Function used to simulate ground truth readings at timestep 'k'
            % Particles are spawned at [250,250] & [-250,-250] and die at [500,-1000] &
            % [1000,-500] after 100s. They meet at 50 s.
            %  A third particle is spawned after 66s at [575,-415] and dies at 100s.
                        %
                        %   Inputs:
                        %       obj     - object data consisting current timestep 's'
                        %
                        %   Outputs:
                        %       obj.g     - ground truth reading at time 't' s
                        %
                        %   Author: Sidddharth Ratapani Navin
                        %   Date:   10/28/2018
                obj.g(1,:)=[-250+12.7475*obj.t*0.9806;-250+12.7475*obj.t*-0.1961]; % Sample sensor readings from the normal distrubtion with sigma=10 m/s^2
                obj.g(2,:)=[250+12.7475*obj.t*0.1961;250+12.7475*obj.t*-0.9806];
                if(obj.t>66)
                    obj.g(3,:)=[417.4+12.7475*(obj.t-67)*-0.92;-587+12.7475*(obj.t-67)*-0.39];
                end
        end
        function [flow_x,flow_y]=optical_flow(obj,car_iter)
            flow_image_x=obj.flow_list.flow(obj.t_steps).Vx;
            flow_image_y=obj.flow_list.flow(obj.t_steps).Vx;
            box=obj.bbox{obj.t_steps}(car_iter,:);
            box(3)=box(3)-box(1);
            box(4)=box(4)-box(2);
            flow_x=mean(imcrop(flow_image_x,box),all);
            flow_y=mean(imcrop(flow_image_y,box),all);
        end
        function RCNNSensor(obj)
            box=obj.bbox{obj.t};
            centres(:,1)=(box(:,1)+box(:,3))/2;
            centres(:,2)=(box(:,2)+box(:,4))/2;
            obj.z=centres;
        end
        function normalize_weights(obj)
            sum_weight=0;
            for iter=1:obj.J_k
                sum_weight=sum_weight+obj.x_k(iter).weight;
            end
            for iter=1:obj.J_k
                obj.x_k(iter).weight=obj.x_k(iter).weight/sum_weight;
            end     
        end
        %
        function PruningAndMerging(obj)
        % Function used to Prune and Merge the updated state estimates obj.x_k 
                    %
                    %   Inputs:
                    %       obj.x_k     - Current updated state esimate at timestep k
                    %       T       - Truncation threshold
                    %       U       - Merging threshold for Mahalanobis distance
                    %       J_max   - Maximum number of particles allowed in the
                    %                 pruned state estimate
                    %
                    %   Outputs:
                    %       Pruned-obj.x_k       - Pruned state estimate
                    %   Author: Siddharth Ratapani Navin
                    %   Date:   10/28/2018
        %% TABLE 2 - Pruning for the Guassian Mixture PHD Filter
            %% Find indices of all weights above the threshold weight T

            index=1;
            for iter=1:length(obj.x_k)
                if(obj.x_k(iter).weight>obj.T)
                    I(index)=iter;
                    index=index+1;
                end    
            end
            l=0;

            %% Merging Loop for Merging state estimates with a Mahalanobis Distance < U

            while(~isempty(I))
                l=l+1;
                max_weight=0;
                j=0;
                % Find index of max weight
                for iter=1:length(I)
                    if(obj.x_k(I(iter)).weight>max_weight)
                        max_weight=obj.x_k(I(iter)).weight;
                        j=I(iter);
                    end
                end

                % Find set of objects that you can merge
                merge_set_size=1;
                L=[];
                for iter=1:length(I)
                    % Calculate Mahalanobis Distance between each object and the
                    % object with max weight to merge it
                    Mah_d=(obj.x_k(I(iter)).mu-obj.x_k(j).mu)'*(obj.x_k(I(iter)).P\(obj.x_k(I(iter)).mu-obj.x_k(j).mu));
                    % Each set L consists of all the indices of objects with
                    % similiar estimates
                    if(Mah_d<=obj.U)
                        L(merge_set_size)=I(iter);
                        merge_set_size=merge_set_size+1;
                    end
                end
                % Calculate weight for new merged member
                Pruned_obj.x_k(l).weight=0;
                for iter=1:length(L)
                    Pruned_obj.x_k(l).weight=Pruned_obj.x_k(l).weight+obj.x_k(L(iter)).weight;
                end
                %Calculate the state vector for the merged member by taking a
                %weighted mean
                Pruned_obj.x_k(l).mu=zeros(4,1);
                for iter=1:length(L)
                    Pruned_obj.x_k(l).mu=Pruned_obj.x_k(l).mu+obj.x_k(L(iter)).weight*obj.x_k(L(iter)).mu;
                end
                Pruned_obj.x_k(l).mu=Pruned_obj.x_k(l).mu/Pruned_obj.x_k(l).weight;
                %Calculate the Covariance of the merged member
                Pruned_obj.x_k(l).P=zeros(4,4);
                for iter=1:length(L)
                    Pruned_obj.x_k(l).P=Pruned_obj.x_k(l).P + obj.x_k(L(iter)).weight*(obj.x_k(L(iter)).P + (Pruned_obj.x_k(l).mu-obj.x_k(L(iter)).mu)*(Pruned_obj.x_k(l).mu-obj.x_k(L(iter)).mu)');
                end
                Pruned_obj.x_k(l).P=Pruned_obj.x_k(l).P/Pruned_obj.x_k(l).weight;
                % Remove the merged members from the original array of Indices I
                I=setdiff(I,L);

            end
            %% Discarding state estimates if the total number of estimates > 100 
            % Check if number of objects >100 and select only the elements with the
            % top 100 weights
            if(length(Pruned_obj.x_k)>obj.J_max)
                Pruned_obj.x_k=nestedSortStruct(Pruned_obj.x_k, 'weight');
                Pruned_obj.x_k=Pruned_obj.x_k(end-99:end);
            end
            obj.x_k=Pruned_obj.x_k;
            
        end
        %
        function MultiTargetStateExtraction(obj)
            % Function used to Extract the final state of the objects at timestep 'k'
                        %
                        %   Inputs:
                        %       obj.x_k     - current state estimate at time-step k
                        %
                        %   Outputs:
                        %       Extracted_x - state estimate after weight thresholding
                        %
                        %   Author: Sidddharth Ratapani Navin
                        %   Date:   10/28/2018
            %% Table 3 - Multi-Target State Extraction
            l=1;
            for iter=1:length(obj.x_k)
                % Check if weight is greater than preset threshold
                if(obj.x_k(iter).weight>0.0001)
                    Extracted_x(l)=obj.x_k(iter);
                    l=l+1;
                end
            end
            obj.x_k=Extracted_x;  
        end
        %
        function run(obj)
            figure('pos',[1500 1500 1500 1500]);
            axis([-500 1200 -1200 500])
            hold on
            obj.ground_truth=zeros(3,2,100);
            %% Main Loop for a 100 timesteps
            for k=1:obj.t_steps
                obj.t=k;
                obj.i=0; 
                %% Step 1.
                % Give Birth to new objects
                if(obj.type==0)
                    for it1=1:2
                       for it2=1:length(obj.x_k)
                           obj.i=obj.i+1;
                           obj.x_pred(obj.i).weight=BirthWeight(obj.x_k(it2).mu);
                           obj.x_pred(obj.i).mu=obj.mu_gamma(:,it1);
                           obj.x_pred(obj.i).P=obj.P_gamma;
                       end
                    end
                else
                    obj.x_pred=CarBirth();
                end
                % Spawning
                for it2=1:obj.J_beta
                    for it3=1:obj.J_k
                        obj.i=obj.i+1;
                        obj.x_pred(obj.i)= SpawnMotionModel(obj.x_k(it3));
                        obj.x_pred(obj.i).weight=SpawnWeight(obj.x_pred(obj.i).mu,obj.x_k(it3).mu)*obj.x_k(it3).weight;
                    end
                end

                %% Step 2.
                %Prediction using existing targets

                for it4=1:obj.J_k
                    obj.i=obj.i+1;
                    obj.x_pred(obj.i)=ExistingObjectMotionModel(obj.x_k(it4));
                end

                J_kpred=obj.i; %Predicted number of targets

                 %% Step 3.
                % Construct Update for PHD components

                for it5=1:J_kpred
                    [a,b]=UpdatePHDComponents(obj.x_pred(it5));
                    %[obj.PHDupdate(it5),obj.x_pred(it5).P]= UpdatePHDComponents(obj.x_pred(it5));
                    obj.PHDupdate(it5)=a;
                    obj.x_pred(it5).P=b;
                end

                %% Step 4.
                % Update
                % Missed Detections

                for it6=1:J_kpred
                    obj.x_k(it6)=ObjectMissedDetection(obj.x_pred(it6));
                end

                % Detected Objects

                l=0; %To iterate all object over all sensor readings
                if(obj.type==0)
                    obj.positionSensor;
                    obj.groundTruth;
                    obj.ground_truth(1:length(obj.z),:,k)=obj.g;
                else
                    obj.RCNNSensor;
                end
                for it7=1:length(obj.z)
                    l=l+1;
                    for it8=1:J_kpred
                        obj.x_k((l-1)*J_kpred+it8)= DetectedObjectUpdate(obj.x_pred(it8),obj.z(it7,:),obj.PHDupdate(it8));
                    end
                end
                obj.J_k=l*J_kpred; %Update the number of particles

                %% Normalize the weights
                obj.normalize_weights()
                %% Pruning and Merging
                obj.PruningAndMerging();
                %% Multi Target State Extraction
                obj.MultiTargetStateExtraction();
                obj.J_k=length(obj.x_k);
                %% Plot Trajectories
                if(obj.type==0)
                    for iter=1:length(obj.x_k)
                        obj.traj(iter).mu(:,k)=obj.x_k(iter).mu;
                        plot(obj.traj(iter).mu(1,1:end),obj.traj(iter).mu(2,1:end),'r*','LineWidth',0.15);
                        hold on 
                        plotcov2d(obj.x_k(iter).mu(1),obj.x_k(iter).mu(2),obj.x_k(iter).P,'b',0);
                        hold on
                    end
                    plot(reshape(obj.ground_truth(1,1,1:k),1,k),reshape(obj.ground_truth(1,2,1:k),1,k),'g-','LineWidth',1.1);
                    hold on
                    plot(reshape(obj.ground_truth(2,1,1:k),1,k),reshape(obj.ground_truth(2,2,1:k),1,k),'g-','LineWidth',1.1);
                    if(k>66)
                    hold on
                    plot(reshape(obj.ground_truth(3,1,67:k),1,k-67+1),reshape(obj.ground_truth(3,2,67:k),1,k-67+1),'g-','LineWidth',1.1);
                    end
                    legend('Estimate','Covariance','Ground Truth');
                    pause(0.05);
                    %EOFor
                else
                    images=load("Test/im_list.mat");
                    imshow("Test/Images/"+images.im_list(obj.t)+".jpg");
                    hold on
                    for iter=1:length(obj.x_k)    
                        plotcov2d(obj.x_k(iter).mu(1),obj.x_k(iter).mu(2),obj.x_k(iter).P,'b',0);
                        hold on
                        box=obj.bbox{obj.t};
%                         for iter1=1:size(box,1)
%                             centres(1)=(box(iter1,1)+box(iter1,3))/2;
%                             centres(2)=(box(iter1,2)+box(iter1,4))/2;
%                             rectangle('Position', [0.8*centres(1),centres(2),(box(iter1,3)-box(iter1,1)),(box(iter1,4)-box(iter1,2))],'EdgeColor','r','LineWidth',2)
%                         end
                        length(obj.x_k) 
                        pause(0.1);
                    end
                end
            end  
        end
    end
end