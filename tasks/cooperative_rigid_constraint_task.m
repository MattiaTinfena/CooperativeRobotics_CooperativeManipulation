classdef cooperative_rigid_constraint_task < Task
    properties
        xt_dot_coop;
        C;
        Hrl;
    end

    methods
        function obj=cooperative_rigid_constraint_task(robot_ID,taskID, smooth)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.smooth=smooth;
        end

        function updateReference(obj, robot_system)

            xt_dot_feas = obj.Hrl * (eye(12)-(pinv(obj.C) * obj.C)) * [obj.xt_dot_coop; obj.xt_dot_coop];
            obj.xdotbar = xt_dot_feas(1:6);
        end

        function updateJacobian(obj,robot_system)

            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end

            obj.J = robot.wJt;
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end