%% ROBOT DYNAMICS AND CONTROL ASSIGNMENT 2 %%%%
%% Perri Alessandro - s4476726
%Algorithm for Inverse Dynamic Problem
%Args: - robot (struct of two structs which contains geometric and inertial parameters of the robot (first struct) and the values of the motion snapshot (second struct))
%      - F_ext (external forces, set a null array of dimension (3xn_links) if none)
%      - M_ext (external torques, set a null array of dimension (3xn_links) if none)
%      - g (gravity acceleration vector, set a null array of dimension (3x1) if none)

function [tau] = invDyn(robot,F_ext,M_ext,g)    
    for i = 1:max(size(robot.Data))
        %Computing rotation matrices
        R_i=robot.Data(i).axangrot; %fixed rotation between frames
        Rz_q(:,:,i)=axang2rotm([0 0 1 robot.Config(i).q]); %rotation about z in the specific configuration (function of q)

        

        %FORWARD RECURSION (all values are seen by the base frame) 
        if(robot.Data(i).type == "revolute")
                R(:,:,i)=R_i*Rz_q(:,:,i);
                if i == 1 %first iteration
                    R_abs(:,:,i)=R(:,:,i);
                    r(:,i)=R_abs(:,:,i)*robot.Data(i).poswrtparent; % r_i/i-1 [m]
                    w(:,i)= R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qd); %angular velocity of Pi wrt 0 [rad/s]
                    wd(:,i)= R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qdd); %angular acceleration of Pi wrt 0 [rad/s^2]
                    v(:,i)=[0;0;0]; %linear velocity of of Pi wrt 0 [m/s]
                    vd(:,i)=[0;0;0]; %linear acceleration of Pi wrt 0 [m/s^2]
                       
                else
                    R_abs(:,:,i)=R_abs(:,:,i-1)*R(:,:,i);
                    r(:,i)=R_abs(:,:,i-1)*robot.Data(i).poswrtparent; % r_i/i-1 [m]
                    w(:,i)= w(:,i-1) + R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qd); %angular velocity of Pi wrt 0 [rad/s]
                    wd(:,i)=wd(:,i-1)+ cross(w(:,i-1),R_abs(:,:,i)*[0;0;1])*robot.Config(i).qd+R_abs(:,:,i)*[0; 0; 1]*robot.Config(i).qdd; %angular acceleration of Pi wrt 0 [rad/s^2]
                    v(:,i)=v(:,i-1) + cross(w(:,i-1),r(:,i)); %linear velocity of Pi wrt 0 [m/s] 
                    vd(:,i)=vd(:,i-1)+ cross(wd(:,i-1),r(:,i))+cross(w(:,i-1),cross(w(:,i-1),r(:,i))); %linear acceleration of Pi wrt 0 [m/s^2]
                end
                
        elseif(robot.Data(i).type == "prismatic") 
            R(:,:,i)=R_i;
            
                if i == 1 %first iteration
                    R_abs(:,:,i)=R(:,:,i);
                    r(:,i)=R_abs(:,:,i)*robot.Data(i).poswrtparent + R_abs(:,:,i)*[0; 0; 1]*robot.Config(i).q; % r_i/i-1 [m]
                    w(:,i)= [0; 0; 0]; %angular velocity of i-th joint [rad/s]
                    wd(:,i)= [0; 0; 0]; %angular acceleration of i-th joint [rad/s^2]
                    v(:,i)=R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qd); %linear velocity of i-th joint [m/s] 
                    vd(:,i)=R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qdd); %linear acceleration of i-th joint [m/s^2]

                else
                    R_abs(:,:,i)=R_abs(:,:,i-1)*R(:,:,i);
                    r(:,i)=R_abs(:,:,i-1)*robot.Data(i).poswrtparent + R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).q); % r_i/i-1 [m]
                    w(:,i)= w(:,i-1); %angular velocity of i-th joint [rad/s]
                    wd(:,i)= wd(:,i-1); %angular acceleration of i-th joint [rad/s^2]
                    v(:,i)= v(:,i-1) + cross(w(:,i-1),r(:,i))+R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qd); %linear velocity of i-th joint [m/s] 
                    vd(:,i)= vd(:,i-1)+ cross(wd(:,i-1),r(:,i))+cross(w(:,i-1),cross(w(:,i-1),r(:,i))) + 2*cross(w(:,i-1),R_abs(:,:,i)*([0;0;1]*robot.Config(i).qd))+R_abs(:,:,i)*([0; 0; 1]*robot.Config(i).qdd); %linear acceleration of i-th joint [m/s^2]
                end
        end


        end
        
       %BACWARD RECURSION 
       for i = max(size(robot.Data)):-1:1
            a_cm(:,i)=vd(:,i) + cross(wd(:,i),(R_abs(:,:,i)*robot.Data(i).CoM')) + cross(w(:,i),cross(w(:,i),R_abs(:,:,i)*robot.Data(i).CoM')); %acceleration of CoM of i-th link
            D(:,i)=robot.Data(i).mass * a_cm(:,i); %dynamic force 
            Delta(:,i)= R_abs(:,:,i)*robot.Data(i).I*R_abs(:,:,i)'* wd(:,i) + cross(w(:,i),R_abs(:,:,i)*robot.Data(i).I*R_abs(:,:,i)'*w(:,i)); %dynamic torque
            
            if i == max(size(robot.Data)) %Fi_i-1 and Mi_i-1 computation
                Fi(:,i)=D(:,i) - robot.Data(i).mass*g - F_ext(:,i);
                Mi(:,i)=Delta(:,i) - M_ext(:,i) - cross(-R_abs(:,:,i)*robot.Data(i).CoM',Fi(:,i));
            else
                Fi(:,i)=D(:,i) - robot.Data(i).mass*g - F_ext(:,i) + Fi(:,i+1) ;
                Mi(:,i)=Delta(:,i) - M_ext(:,i) - cross(-R_abs(:,:,i)*robot.Data(i).CoM',Fi(:,i)) + Mi(:,i+1) + cross((r(:,i+1)-R_abs(:,:,i)*robot.Data(i).CoM'),Fi(:,i+1));
                
            end
            

            %Computation of tau_i
            if(robot.Data(i).type == 'revolute')
                tau(i)=Mi(:,i)'*R_abs(:,:,i)*[0;0;1];

            elseif(robot.Data(i).type == 'prismatic')
                tau(i)=Fi(:,i)'*R_abs(:,:,i)*[0;0;1];

            end

       end



end

   
