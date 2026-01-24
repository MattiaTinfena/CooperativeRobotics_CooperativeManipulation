classdef bimanual_rigid_constraint_task < Task
    properties

    end

    % TODO: MUST BE MODIFIED TO BE A COOPERATIVE AND NOT A BIMANUAL

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

            robot_left=robot_system.left_arm;

            robot_right=robot_system.right_arm;

            obj.J = [robot_left -robot_right];
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end