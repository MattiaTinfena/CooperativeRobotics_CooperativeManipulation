classdef bimanual_rigid_constraint_task < Task
    properties

    end

    methods
        function obj=bimanual_rigid_constraint_task(robot_ID,taskID, smooth)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.smooth=smooth;
        end

        function updateReference(obj, robot_system)

            obj.xdotbar = zeros(6,1);
        end

        function updateJacobian(obj,robot_system)

            obj.J = [robot_system.left_arm.wJt, -robot_system.right_arm.wJt];
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end